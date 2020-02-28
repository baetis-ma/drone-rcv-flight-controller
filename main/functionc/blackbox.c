
int cnt = 0;
void blackbox_string() {
    int temp_int;
    cnt++;
    blackbox_str[0] = cnt/256; blackbox_str[1] = cnt%256;
    blackbox_str[2] = (uint8_t)128+height;
    blackbox_str[3] = (uint8_t)128+heightprog;
    blackbox_str[4] = (uint8_t)(heading/2);
    temp_int = (360 + (int)headingprog)%360;
    blackbox_str[5] = (uint8_t)(temp_int/2);
    blackbox_str[6] = (uint8_t)128+xdisp; 
    blackbox_str[7] = (uint8_t)128+ydisp;
    temp_int = (int)(100 * (180 + imu.pitch));
    blackbox_str[8] = (int8_t)(temp_int/256);
    blackbox_str[9] = (int8_t)(temp_int%256);
    temp_int = (int)(100 * (180 + imu.roll));
    blackbox_str[10] = (int8_t)(temp_int/256);
    blackbox_str[11] = (int8_t)(temp_int%256);
    blackbox_str[12] = Motor1/256; blackbox_str[13] = Motor1%256;
    blackbox_str[14] = Motor2/256; blackbox_str[15] = Motor2%256;
    blackbox_str[16] = Motor3/256; blackbox_str[17] = Motor3%256;
    blackbox_str[18] = Motor4/256; blackbox_str[19] = Motor4%256;
    blackbox_str[20] = (int8_t)10.0*(meas_battery() - 10);
}
