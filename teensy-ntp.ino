#include "lwip_t41.h"
#include "lwip/inet.h"
#include "lwip/dhcp.h"
#include "InputCapture.h"
#include "DateTime.h"
#include "GPS.h"
#include "NTPClock.h"
#include "ClockPID.h"
#include "NTPServer.h"
#include "NTPClients.h"
#include "platform-clock.h"
#include "WebServer.h"
#include "WebContent.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "maidenhead.h"

// see the settings file for common settings
#include "settings.h"

#define WAIT_COUNT 3
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

GPSDateTime gps(&GPS_SERIAL);
NTPClock localClock;
NTPClients clientList;
InputCapture pps;
elapsedMillis msec, epoll_msec;
uint32_t compileTime;
uint8_t settime = 0;
uint8_t wait = WAIT_COUNT-1;
struct {
  int64_t offset;
  uint32_t pps;
  uint32_t gpstime;
} samples[WAIT_COUNT];
NTPServer server(&localClock);

static void netif_status_callback(struct netif *netif) {
  char str1[IP4ADDR_STRLEN_MAX] = "", str2[IP4ADDR_STRLEN_MAX] = "", str3[IP4ADDR_STRLEN_MAX] = "";
  const ip_addr_t *ip;

  ip = netif_ip_addr4(netif);
  ip4addr_ntoa_r(ip_2_ip4(ip), str1, IP4ADDR_STRLEN_MAX);

  ip = netif_ip_netmask4(netif);
  ip4addr_ntoa_r(ip_2_ip4(ip), str2, IP4ADDR_STRLEN_MAX);

  ip = netif_ip_gw4(netif);
  ip4addr_ntoa_r(ip_2_ip4(ip), str3, IP4ADDR_STRLEN_MAX);
  Serial.printf("netif status changed: ip %s, mask %s, gw %s\r\n", str1, str2, str3);

#if LWIP_IPV6
  for(int i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i++) {
    char str6[IP6ADDR_STRLEN_MAX] = "";
    if (netif_ip6_addr_state(netif, i) != 0) {
      ip = netif_ip_addr6(netif, i);
      ip6addr_ntoa_r(ip_2_ip6(ip), str6, IP6ADDR_STRLEN_MAX);
      Serial.printf("v6: %s state %d\r\n", str6, netif_ip6_addr_state(netif, i));
    }
  }
#endif
}

static void link_status_callback(struct netif *netif) {
  Serial.printf("enet link status: %s\r\n", netif_is_link_up(netif) ? "up" : "down");
  if (netif_is_link_up(netif)) {
    netif_set_up(netif);
    dhcp_start(netif);
#if LWIP_IPV6
    netif_create_ip6_linklocal_address(netif, 1);
    netif_set_ip6_autoconfig_enabled(netif, 1);
#endif
  }
}

void wait_for_serial() {
  // don't wait forever in case usb doesn't come up
  for (int i = 0; i < 20; i++) {
    if (Serial) return;
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);

  wait_for_serial();

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.display();

   display.ssd1306_command(SSD1306_SETCONTRAST);
   display.ssd1306_command(255); //1=lowest, 255=highest
   display.ssd1306_command(SSD1306_SETVCOMDETECT);
   display.ssd1306_command(0x3C);//default 0x30 per datasheet
  // //Additionally the precharge control register D9 will add additional dimming, use with care and monitor the current.
  // display.ssd1306_command(SSD1306_SETPRECHARGE);
  // display.ssd1306_command(0xff); //bits 7..4 control phase 1, bits 3..0 phase 2, use with care as you can increase the power draw. 0x11 makes it quite dim, 0xA1 is much brighter.

  Serial.println("Ethernet 1588 NTP Server");
  Serial.println("------------------------\n");

  DateTime compile = DateTime(__DATE__, __TIME__);

  GPS_SERIAL.begin(GPS_BAUD);

  enet_init(NULL, NULL, NULL);

  netif_set_status_callback(netif_default, netif_status_callback);
  netif_set_link_callback(netif_default, link_status_callback);
  netif_set_hostname(netif_default, DHCP_HOSTNAME);

  pps.begin();
  server.setup();

  compileTime = compile.ntptime();
  // this needs to happen after enet_init, so the 1588 clock is running
  localClock.setTime(COUNTERFUNC(), compileTime);
  // allow for compile timezone to be 14 hours ahead
  compileTime -= 14*60*60;

  webserver.begin();
  webcontent.begin();

  while(GPS_SERIAL.available()) { // throw away all the text received while starting up
    GPS_SERIAL.read();
  }
  msec = 0;
  epoll_msec = 0;
}

static uint8_t median(int64_t one, int64_t two, int64_t three) {
  if(one > two) {
    if(one > three) {
      if(two > three) {
        // 1 2 3
        return 2-1;
      } else {
        // 1 3 2
        return 3-1;
      }
    } else {
      // 3 1 2
      return 1-1;
    }
  } else {
    if(two > three) {
      if(one > three) {
        // 2 1 3
        return 1-1;
      } else {
        // 2 3 1
        return 3-1;
      }
    } else {
      // 3 2 1
      return 2-1;
    }
  }
}

static uint32_t ntp64_to_32(int64_t offset) {
  if(offset < 0)
    offset *= -1;
  // take 16bits off the bottom and top
  offset = offset >> 16;
  return offset & 0xffffffff;
}

void updateTime(uint32_t gpstime) {
  if(gps.ppsMillis() == 0) {
    return;
  }

  uint32_t ppsToGPS = gps.capturedAt() - gps.ppsMillis();
  webcontent.setPPSData(ppsToGPS, gps.ppsMillis(), gpstime);
  if(ppsToGPS > 950) { // allow 950ms between PPS and GPS message
    Serial.print("LAG ");
    Serial.print(ppsToGPS);
    Serial.print(" ");
    Serial.print(gps.ppsMillis());
    Serial.print(" ");
    Serial.println(gpstime);
    return;
  }

  uint32_t lastPPS = gps.ppsCounter();
  if(settime) {
    int64_t offset = localClock.getOffset(lastPPS, gpstime, 0);
    samples[wait].offset = offset;
    samples[wait].pps = lastPPS;
    samples[wait].gpstime = gpstime;
    if(ClockPID.full() && wait) {
      wait--;
    } else {
      uint8_t median_index = wait;
      if(wait == 0) {
        median_index = median(samples[0].offset, samples[1].offset, samples[2].offset);
      }
      ClockPID.add_sample(samples[median_index].pps, samples[median_index].gpstime, samples[median_index].offset);
      localClock.setRefTime(samples[median_index].gpstime);
      localClock.setPpb(ClockPID.out() * 1000000000.0);
      wait = WAIT_COUNT-1; // (2+1)*16=48s, 80MHz wraps at 53s

      // TODO: this should grow when out of sync
      server.setDispersion(ntp64_to_32(samples[median_index].offset));
      server.setReftime(samples[median_index].gpstime);

      double offsetHuman = samples[median_index].offset / (double)4294967296.0;
      webcontent.setLocalClock(samples[median_index].pps, offsetHuman, ClockPID.d(), ClockPID.d_chi(), localClock.getPpb(), gpstime);
      Serial.print(samples[median_index].pps);
      Serial.print(" ");
      Serial.print(offsetHuman, 9);
      Serial.print(" ");
      Serial.print(ClockPID.d(), 9);
      Serial.print(" ");
      Serial.print(ClockPID.d_chi(), 9);
      Serial.print(" ");
      Serial.print(localClock.getPpb());
      Serial.print(" ");
      Serial.println(samples[median_index].gpstime);
    }
  } else {
    localClock.setTime(lastPPS, gpstime);
    ClockPID.add_sample(lastPPS, gpstime, 0);
    settime = 1;
    Serial.print("S "); // clock set message
    Serial.print(lastPPS);
    Serial.print(" ");
    Serial.println(gpstime);
  }
}

static void slower_poll() {
  if (epoll_msec >= 100) {
    // check link state, update dhcp, etc
    enet_poll();

    epoll_msec = 0;
  }

  if(msec >= 1000) {
    uint32_t s, s_fb;
    // update the local clock's cycle count
    localClock.getTime(COUNTERFUNC(),&s,&s_fb);

    // remove old NTP clients
    clientList.expireClients();

    msec = 0;
  }
}

static void gps_serial_poll() {
  if(GPS_SERIAL.available()) {
    if(gps.decode()) {
      uint32_t gpstime = gps.GPSnow().ntptime();
      if(gpstime < compileTime) {
        Serial.print("B "); // gps clock bad message (for example, on startup before GPS almanac)
        Serial.println(gpstime);
      } else {
        updateTime(gpstime);
      }
    }
  }
}

// useful when using teensy_loader_cli
static void bootloader_poll() {
  if(Serial.available()) {
    char r = Serial.read();
    if(r == 'r') {
      Serial.println("rebooting to bootloader");
      delay(10);
      asm("bkpt #251"); // run bootloader
    }
  }
}

uint32_t nextUpdate = 0, mainLoopCycles = 0;

void display_service(void)
{
    mainLoopCycles++;

  if (millis()>=nextUpdate)
  {
    Serial.print("MainLoop, ");
    Serial.print(mainLoopCycles);
    Serial.println(" cycles.");
    char localTime[100], utcTime[100], maidenhead[100], displayString[300], *mhp;

    mhp = gps.getMaidenhead();
    strcpy(maidenhead,mhp);
    maidenhead[4] = tolower(maidenhead[4]);
    maidenhead[5] = tolower(maidenhead[5]);

    sprintf(utcTime,"%02d:%02d:%02d",gps.GPSnow().hour(),gps.GPSnow().minute(),gps.GPSnow().second());
    int localHour = gps.GPSnow().hour()+UTC_LOCAL_OFFSET;
    if (localHour < 0) localHour+=24;
    sprintf(localTime,"%02d:%02d:%02d",localHour,gps.GPSnow().minute(),gps.GPSnow().second());

    sprintf(displayString,"%s\n%s\n%s",localTime,utcTime,maidenhead);
    Serial.println(displayString);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font

    display.setCursor(0, 0);     // Start at top-left corner
    display.setTextSize(2);      // 2:1 pixel scale
    display.write(localTime);
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.write("Local");

    display.setCursor(0, 20);
    display.setTextSize(2);
    display.write(utcTime);
    display.setTextSize(1);
    display.write("UTC");

    display.setCursor(0, 40);
    display.setTextSize(2);
    display.write(maidenhead);

    display.display();    

    nextUpdate += 1000;
    mainLoopCycles = 0;
  }
}

void loop() {
  enet_proc_input();

  slower_poll();

  enet_proc_input();

  gps_serial_poll();

  enet_proc_input();

  bootloader_poll();

  display_service();
}
