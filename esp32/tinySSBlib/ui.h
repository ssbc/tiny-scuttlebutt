// ui.h

#ifndef _INCLUDE_UI_H
#define _INCLUDE_UI_H


class UIClass {
  
public:
  UIClass();
  virtual ~UIClass();
  virtual void spinner(bool show) {};
  virtual void buzz() {};
  virtual void loop() {};    // for screen animations
  virtual void refresh() {}; // force refresh (of current screen)
  virtual void show_boot_msg(char *msg) { Serial.printf("# %s\r\n", msg); };
  virtual void boot_ended() {};

  // general:
  void show_node_name(char *s);
  void show_time(char *s);
  void show_gps(float lon, float lat, float ele);

  // repo screen:
  void show_repo_stats(int f, int e, int c, int freePercent);

  // comm
  void show_ble_peers(int n);
  void show_wifi_peers(int n);
  void show_lora_specs(char *profile, long fr, int bw, int sf);
  virtual void lora_advance_wheel() {};

  // peers screen:
  virtual void heard_peer(char *id, int rssi, float snr) {};
  
  char *node_name;
  char *time;
  float gps_lon, gps_lat, gps_ele; 
  int f_cnt, e_cnt, c_cnt;
  int ble_cnt, wifi_cnt;
  char *lora_profile;
  long lora_fr;
  int lora_bw, lora_sf;
};


#endif
