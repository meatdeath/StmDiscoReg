/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.44                          *
*        Compiled Nov 10 2017, 08:53:57                              *
*        (c) 2017 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
#include "stm32f429i_discovery_lcd.h"
#include "stdio.h"
#include "main.h"
#include "version.h"
// USER END

#include "DIALOG.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_FRAMEWIN_0 (GUI_ID_USER + 0x00)
#define ID_GRAPH_0 (GUI_ID_USER + 0x01)
#define ID_TEXT_0 (GUI_ID_USER + 0x02)
#define ID_EDIT_0 (GUI_ID_USER + 0x03)
#define ID_PROGBAR_0 (GUI_ID_USER + 0x04)
#define ID_TEXT_1 (GUI_ID_USER + 0x05)
#define ID_TEXT_2 (GUI_ID_USER + 0x06)
#define ID_EDIT_1 (GUI_ID_USER + 0x07)
#define ID_TEXT_3 (GUI_ID_USER + 0x08)


// USER START (Optionally insert additional defines)
// USER END

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)
// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { FRAMEWIN_CreateIndirect, "Framewin", ID_FRAMEWIN_0, 0, 0, 240, 320, 0, 0x64, 0 },
  { GRAPH_CreateIndirect, "Graph", ID_GRAPH_0, 0, 47, 230, 158, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "CurrentText", ID_TEXT_0, 35, 17, 63, 20, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "EditCurrent", ID_EDIT_0, 105, 10, 82, 31, 0, 0x64, 0 },
  { PROGBAR_CreateIndirect, "Progbar", ID_PROGBAR_0, 5, 240, 220, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "USBText", ID_TEXT_1, 5, 220, 94, 20, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "TextA", ID_TEXT_2, 190, 17, 30, 20, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "DateTimeEdit", ID_EDIT_1, 0, 268, 230, 21, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "USBStatusText", ID_TEXT_3, 100, 220, 125, 20, 0, 0x64, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

// USER START (Optionally insert additional static code)
extern void myprintf(const char *fmt, ...);
// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;
  // USER START (Optionally insert additional variables)
  // USER END

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'Framewin'
    //
    hItem = pMsg->hWin;
    FRAMEWIN_SetText(hItem, "Registrator");
    FRAMEWIN_SetTitleHeight(hItem, 24);
    FRAMEWIN_SetFont(hItem, GUI_FONT_20_ASCII);
    //
    // Initialization of 'Graph'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_GRAPH_0);
    GRAPH_SetBorder(hItem, 32, 2, 2, 2);
    //
    // Initialization of 'CurrentText'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
    TEXT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
    TEXT_SetText(hItem, "Current");
    //
    // Initialization of 'EditCurrent'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
    EDIT_SetText(hItem, "0");
    EDIT_SetFont(hItem, GUI_FONT_32B_ASCII);
    EDIT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
    //
    // Initialization of 'USBText'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    TEXT_SetText(hItem, "USB status");
    TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
    //
    // Initialization of 'TextA'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
    TEXT_SetText(hItem, "A");
    TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
    //
    // Initialization of 'DateTimeEdit'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_1);
    EDIT_SetText(hItem, "2021/04/29   17:48:27");
    EDIT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
    EDIT_SetFont(hItem, GUI_FONT_20_ASCII);
    //
    // Initialization of 'USBStatusText'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
    TEXT_SetText(hItem, "Not connected");
    TEXT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
    TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
    TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x000000FF));
    // USER START (Optionally insert additional code for further widget initialization)
    // USER END
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_EDIT_0: // Notifications sent by 'EditCurrent'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_1: // Notifications sent by 'DateTimeEdit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
    	  events |= EVT_DATE_TIME_DLG;
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateFramewin
*/
WM_HWIN CreateFramewin(void);
WM_HWIN CreateFramewin(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

// USER START (Optionally insert additional public code)
#define GRAPH_DATA_SIZE	200
WM_HWIN hMainWin;
WM_HWIN hEditCurrent;
WM_HWIN hEditDateTime;
WM_HWIN hUsbStatusText;
WM_HWIN hProgBar;
WM_HWIN hGraph;
WM_HWIN hScale;
int16_t graphLowData[GRAPH_DATA_SIZE] = {100};
int16_t graphHighData[GRAPH_DATA_SIZE] = {0};
GRAPH_DATA_Handle hGraphLowDataObj;
GRAPH_DATA_Handle hGraphHighDataObj;

const int16_t graphVSize = 150;
const int16_t sigAmplitude = 2*3000;
const int16_t scaleFactor = sigAmplitude/graphVSize;

void CreateDialog(void) {
	hMainWin = CreateFramewin();
	FRAMEWIN_SetClientColor(hMainWin,LCD_COLOR_LIGHTGRAY);


    char win_name[30];
    sprintf( win_name, "Registrator (Build %d)", BUILD_VERSION );
    FRAMEWIN_SetText(hMainWin, win_name);

	hEditCurrent = WM_GetDialogItem(hMainWin, ID_EDIT_0);
	hEditDateTime = WM_GetDialogItem(hMainWin, ID_EDIT_1);
	hProgBar = WM_GetDialogItem(hMainWin, ID_PROGBAR_0);
	hUsbStatusText = WM_GetDialogItem(hMainWin, ID_TEXT_3);
	hGraph = WM_GetDialogItem(hMainWin, ID_GRAPH_0);

	//EDIT_SetDecMode(hEditCurrent,0,-2500,2500,0,GUI_EDIT_SIGNED);

	GRAPH_SetGridVis(hGraph,1);
	hGraphLowDataObj = GRAPH_DATA_YT_Create(GUI_GREEN,GRAPH_DATA_SIZE, graphLowData, 0);
	GRAPH_AttachData(hGraph, hGraphLowDataObj);

	hGraphHighDataObj = GRAPH_DATA_YT_Create(GUI_GREEN,GRAPH_DATA_SIZE, graphHighData, 0);
	GRAPH_AttachData(hGraph, hGraphHighDataObj);


	GRAPH_SetGridDistY(hGraph, 1000/scaleFactor);

	GRAPH_DATA_YT_SetOffY(hGraphLowDataObj, graphVSize/2);
	GRAPH_DATA_YT_SetOffY(hGraphHighDataObj, graphVSize/2);

	hScale = GRAPH_SCALE_Create(28, GUI_TA_RIGHT, GRAPH_SCALE_CF_VERTICAL, 1000/scaleFactor);
	GRAPH_AttachScale(hGraph, hScale);
	GRAPH_SCALE_SetFactor(hScale, scaleFactor);
	GRAPH_SCALE_SetOff(hScale, graphVSize/2);
	//GRAPH_SCALE_SetTickDist(hScale, 1000/scaleFactor);
	GRAPH_SCALE_SetTextColor(hScale, GUI_BLACK);
}

void DialogProcess(void) {
	//GUI_Delay(100); // for multitasking
	GUI_Exec();
	WM_PaintWindowAndDescs(hMainWin);
}

extern const int CURRENT_MAX;

void UpdateCurrentEdit(int16_t current) {
	GUI_COLOR gui_color;

	if( current >= 1900 || current <= -1900 ) {
		gui_color = GUI_RED;
	} else if( current >= 1500 || current <= -1500 ) {
		gui_color = GUI_YELLOW;
	} else {
		gui_color = GUI_WHITE;
	}

    EDIT_SetBkColor(hEditCurrent, EDIT_CI_ENABLED, gui_color);

	char str[10];
	sprintf( str, "%d", current );
	EDIT_SetText(hEditCurrent, str);

}

void UpdateDateTimeEdit(
		uint16_t year, uint8_t month, uint8_t day,
		uint16_t hour, uint8_t min, uint8_t sec) {
	char str[25];
	sprintf(str, "%04d / %02d / %02d    %02d : %02d : %02d", year, month, day, hour, min, sec);
	EDIT_SetText(hEditDateTime, str);
}

void UpdateProgressBar(uint8_t percentage) {
	PROGBAR_SetValue(hProgBar, percentage);
}

void UpdateUsbStatusText(const char *str, GUI_COLOR color) {
	TEXT_SetTextColor(hUsbStatusText, color);
	TEXT_SetText(hUsbStatusText, str);
}

void AddGraphData(int16_t low_value, int16_t high_value) {
	GRAPH_DATA_YT_AddValue(hGraphLowDataObj,low_value/scaleFactor);
	GRAPH_DATA_YT_AddValue(hGraphHighDataObj,high_value/scaleFactor);
}
// USER END

/*************************** End of file ****************************/
