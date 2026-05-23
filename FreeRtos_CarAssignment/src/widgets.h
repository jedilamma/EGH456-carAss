#ifndef DISPLAY_WIDGETS_H
#define DISPLAY_WIDGETS_H

#include "taskman.h"

// ====================== Extern Declarations for All Widgets ======================
// Plot mode
typedef enum
{
    OPT_PLOT_MODE_FILTERED_ONLY = 0,
    OPT_PLOT_MODE_RAW_AND_FILTERED,
    IMU_PLOT_MODE_FILTERED_ONLY,
    IMU_PLOT_MODE_RAW_AND_FILTERED,
    RNG_PLOT_MODE_FILTERED_ONLY,
    RNG_PLOT_MODE_RAW_AND_FILTERED
} PlotMode_t;

// ====================== Motor State ======================
typedef enum
{
    MOTOR_IDLE = 0,
    MOTOR_STARTING,
    MOTOR_RUNNING,
    MOTOR_FAULT
} MotorState_t;

// Main Panels
extern tCanvasWidget g_psPanels[];

// Panel 1 - Inputs
extern tSliderWidget g_psSliders[];
extern tCanvasWidget g_psCheckBoxIndicators[];
extern tCheckBoxWidget g_psCheckBoxes[];
extern tCanvasWidget g_psPushButtonIndicators[];
extern tPushButtonWidget g_psPushButtons[];
extern tPushButtonWidget g_sClearFaultButton;

extern tCanvasWidget g_sAccelValueCanvas;
extern tCanvasWidget g_sDistanceCanvas;

extern tCanvasWidget g_sCoolingStatusCanvas;
extern tCanvasWidget g_sLuxMinValue;
extern tCanvasWidget g_sLuxMaxValue;

extern tCanvasWidget g_sEngineCanvas;
extern void OnEngineCanvasPaint(tWidget *psWidget, tContext *psContext);

void OnCrashTolSliderChange(tWidget *psWidget, int32_t i32Value);
extern tSliderWidget g_sCrashTolSlider;

void OnAccelMaxSliderChange(tWidget *psWidget, int32_t i32Value);
extern tSliderWidget g_sAccelMaxSlider;

void OnDetRangeSliderChange(tWidget *psWidget, int32_t i32Value);
extern tSliderWidget g_sDetRangeSlider;

void OnLuxMinSliderChange(tWidget *psWidget, int32_t i32Value);
extern tSliderWidget g_sLuxMinSlider;

// Panel 2 - Plotting
extern tCanvasWidget g_sCanvas1;
extern tCanvasWidget g_sCanvas2;
extern tCanvasWidget g_sCanvas3;
extern tCanvasWidget g_sCanvas4;
extern tCanvasWidget g_sCanvas5;

extern tCheckBoxWidget g_sPlotModeCheckBox;
extern tCheckBoxWidget g_sPlotModeCheckBox2;

extern tCheckBoxWidget g_sPlotModeCheckBox3;
extern tCheckBoxWidget g_sPlotModeCheckBox4;

extern tCheckBoxWidget g_sPlotModeCheckBox5;
extern tCheckBoxWidget g_sPlotModeCheckBox6;

extern tCheckBoxWidget g_sPlotModeCheckBox7;
extern tCheckBoxWidget g_sPlotModeCheckBox8;


extern volatile uint8_t  g_ui8Hours, g_ui8Minutes, g_ui8Seconds;
extern volatile uint8_t  g_ui8Day, g_ui8Month;
extern volatile uint16_t g_ui16Year;
extern volatile int16_t  g_i16MotorValue;
extern uint16_t g_ui32Speed;
// Bottom Navigation
extern char *g_pcPanei32Names[];

// ====================== Globals (EXTERN only) ======================
extern uint32_t g_ui32Panel;
uint32_t g_ui32ButtonState;
extern tContext sContext;

extern bool g_bOPTDisplayHold;
extern bool g_bIMUDisplayHold;
extern bool g_bRNGDisplayHold;
extern bool g_bDisplayHold;

extern bool g_faultSwitch;
extern bool g_startDisplay;
extern bool g_bFaultAcknowledged;
extern bool g_bFaultButtonVisible;

bool g_bStartButtonVisible;
tCanvasWidget g_sDashboard;
tPushButtonWidget g_sStartButton;
tPushButtonWidget g_psSideButtons[6];


void ShowStartButton(void);
void HideStartButton(void);
// Widget callbacks
void OnIntroPaint(tWidget *psWidget, tContext *psContext);

void OnOPTCanvasPaint(tWidget *psWidget, tContext *psContext);
void OnIMUCanvasPaint(tWidget *psWidget, tContext *psContext);
void OnCanvasPaint(tWidget *psWidget, tContext *psContext);
void OnRangeCanvasPaint(tWidget *psWidget, tContext *psContext);

void OnCheckChange(tWidget *psWidget, uint32_t bSelected);
void OnButtonPress(tWidget *psWidget);
void OnSliderChange(tWidget *psWidget, int32_t i32Value);

void OnOPTPlotModeChange(tWidget *psWidget, uint32_t bSelected);
void OnIMUPlotModeChange(tWidget *psWidget, uint32_t bSelected);
void OnOPTPlotModeChange2(tWidget *psWidget, uint32_t bSelected);
void OnIMUPlotModeChange2(tWidget *psWidget, uint32_t bSelected);

void OnPlotModeChange3(tWidget *psWidget, uint32_t bSelected);
void OnPlotModeChange4(tWidget *psWidget, uint32_t bSelected);

void OnPlotModeChange5(tWidget *psWidget, uint32_t bSelected);
void OnPlotModeChange6(tWidget *psWidget, uint32_t bSelected);
void UpdateFaultPanel(void);

void OnClearFaultButtonPress(tWidget *psWidget);
void ShowAckFaultButton(void);
void HideAckFaultButton(void);

void OnDashCanvasPaint(tWidget *psWidget, tContext *psContext);

void OnNext(tWidget *psWidget);
void OnPrevious(tWidget *psWidget);
void OnDashboardPaint(tWidget *psWidget, tContext *psContext);
void OnStartButtonPress(tWidget *psWidget);
void OnDashButtonPress(tWidget *psWidget);
void OnFaultPanelPaint(tWidget *psWidget, tContext *psContext);
void UpdateFaultPanel(void);
void UpdateMotorStateCanvas(void);
void SetMotorState(MotorState_t newState);
void OnDashboardSliderChange(tWidget *psWidget, int32_t i32Value);
void UpdateTime(void);
void UpdateTemp(tContext *psContext);
// ====================== Motor & Fault Globals ======================

extern uint32_t g_ui32ActiveFaults;
extern uint32_t g_ui32ActiveWarnings;
extern uint32_t g_ui32ActiveSafety;
extern uint32_t g_ui32FilteredBodyAccel;
extern uint32_t g_ui32CurrentDistance;
// ====================== New Widgets (Dashboard) ======================
tCanvasWidget g_sMotorStateCanvas;
tCanvasWidget g_sFaultButtonCanvas;

extern tCanvasWidget g_sFaultPanel;
extern tSliderWidget g_sDashboardSlider;
extern tCanvasWidget g_sTimeCanvas;
extern tCanvasWidget g_sTempCanvas;
extern tCanvasWidget g_sLightValCanvas;
extern tCanvasWidget g_sLightValCanvas2;
extern tCanvasWidget g_sAccelValueCanvas;
extern tSliderWidget g_sCrashSlider;
extern tCanvasWidget g_sDistanceCanvas;
extern tSliderWidget g_sMinDistanceSlider;
extern tCanvasWidget g_sCoolingStatusCanvas;
extern tCanvasWidget g_sCanvasDash;

extern uint32_t g_ui32LuxMinThreshold;
extern uint32_t g_ui32CrashThreshold;
extern uint32_t g_ui32MinSafeDistance;
extern uint32_t g_ui32AccelMaxVal;
extern uint32_t g_ui32DetRangeVal;

#endif // DISPLAY_WIDGETS_H