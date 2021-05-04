/*
 * FramewinDLG.h
 *
 *  Created on: May 3, 2021
 *      Author: Vladimir
 */

#ifndef FRAMEWINDLG_H_
#define FRAMEWINDLG_H_

WM_HWIN CreateFramewin(void);
void CreateDialog(void);
void DialogProcess(void);
void UpdateCurrentEdit(int16_t current);
void UpdateDateTimeEdit(uint16_t year, uint8_t month, uint8_t day, uint16_t hour, uint8_t min, uint8_t sec);
void UpdateTimeEdit(uint16_t hour, uint8_t min, uint8_t sec);
void UpdateProgressBar(uint8_t percentage);
void UpdateUsbStatusText(const char *str, GUI_COLOR color);
void AddGraphData(int16_t low_value, int16_t high_value);

#endif /* FRAMEWINDLG_H_ */
