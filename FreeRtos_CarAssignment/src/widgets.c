#include "taskman.h"
#include "widgets.h"
#include "images.h"
#include "slider.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "utils/uartstdio.h"

static void FormatSignedX100(char *buf, const char *label, int32_t value_x100, const char *unit)
{
    const char *sign = "";
    uint32_t absValue;

    if (value_x100 < 0)
    {
        sign = "-";
        absValue = (uint32_t)(-value_x100);
    }
    else
    {
        absValue = (uint32_t)value_x100;
    }

    usprintf(buf, "%s%s%u.%02u %s",
             label,
             sign,
             (unsigned int)(absValue / 100U),
             (unsigned int)(absValue % 100U),
             unit);
}

// CHANGE: Formats values stored as unsigned x100
static void FormatUnsignedX100(char *buf, const char *label, uint32_t value_x100, const char *unit)
{
    usprintf(buf, "%s%u.%02u %s",
             label,
             (unsigned int)(value_x100 / 100U),
             (unsigned int)(value_x100 % 100U),
             unit);
}

// ====================== Widget Definitions ======================


// ====================== PANEL 1 - ENGINE ======================
Canvas(g_sEngineCanvas, g_psPanels + 1, 0, 0,
       &g_sKentec320x240x16_SSD2119, 10, 40, 220, 200, 
       CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN, 0, ClrGray, 
       0, 0, 0, 0, OnEngineCanvasPaint);

// ====================== ENGINE PANEL SLIDER (Vertical) ======================


tSliderWidget g_sAccelMaxSlider = SliderStruct(&g_sEngineCanvas, 
                                             NULL,                  // Next sibling
                                             0,                     // No children
                                             &g_sKentec320x240x16_SSD2119,
                                             60, 60, 100, 15,       // x, y, width, height → Vertical
                                             0, 20, 10,            // min, max, initial value
                                             (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | 
                                              SL_STYLE_OUTLINE | SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),                                              
                                             ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                                             &g_sFontCm14, "10", 0, 0, OnAccelMaxSliderChange);


tSliderWidget g_sDetRangeSlider = SliderStruct(&g_sEngineCanvas, 
                                             NULL,                  // Next sibling
                                             0,                     // No children
                                             &g_sKentec320x240x16_SSD2119,
                                             60, 100, 100, 15,       // x, y, width, height → Vertical
                                             0, 330, 1000,            // min, max, initial value
                                             (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | 
                                              SL_STYLE_OUTLINE | SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),                                              
                                             ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                                             &g_sFontCm14, "330", 0, 0, OnDetRangeSliderChange);     
                                             
tSliderWidget g_sCrashTolSlider = SliderStruct(&g_sEngineCanvas, 
                                             NULL,                  // Next sibling
                                             0,                     // No children
                                             &g_sKentec320x240x16_SSD2119,
                                             60, 140, 100, 15,       // x, y, width, height → Vertical
                                             0, 100, 50,            // min, max, initial value
                                             (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | 
                                              SL_STYLE_OUTLINE | SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),                                              
                                             ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                                             &g_sFontCm14, "50", 0, 0, OnCrashTolSliderChange);
                                             
tSliderWidget g_sLuxMinSlider = SliderStruct(&g_sEngineCanvas, 
                                             NULL,                  // Next sibling
                                             0,                     // No children
                                             &g_sKentec320x240x16_SSD2119,
                                             60, 180, 100, 15,       // x, y, width, height → Vertical
                                             3, 40, 10,            // min, max, initial value
                                             (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | 
                                              SL_STYLE_OUTLINE | SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),                                              
                                             ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                                             &g_sFontCm14, "10", 0, 0, OnLuxMinSliderChange);  


// Panel 2 - Plotting
Canvas(g_sCanvas2, g_psPanels + 2, &g_sPlotModeCheckBox, 0,     // ← IMPORTANT: point to first child
       &g_sKentec320x240x16_SSD2119,
       30, 50, 200, 140,
       CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN,
       0, ClrGray, 0, 0, 0, 0, OnOPTCanvasPaint);

tCheckBoxWidget g_sPlotModeCheckBox2 = CheckBoxStruct(g_psPanels + 2,
    NULL, 0, &g_sKentec320x240x16_SSD2119,
    30, 200, 65, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Hold",
    0, OnOPTPlotModeChange2);

tCheckBoxWidget g_sPlotModeCheckBox = CheckBoxStruct(g_psPanels + 2,
    &g_sPlotModeCheckBox2, 0, &g_sKentec320x240x16_SSD2119,
    135, 200, 95, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Graph Switch",
    0, OnOPTPlotModeChange);
    
// ====================== TIME CANVAS ON DASHBOARD ======================
tCanvasWidget g_sLightValCanvas = CanvasStruct( &g_sCanvas2, 0, 0,
                                            &g_sKentec320x240x16_SSD2119,
                                            0, 40, 28, 50,                                      // x, y, w, h
                                            CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_OPAQUE,
                                            ClrBlack, 0, ClrWhite, &g_sFontCm12, "2000", 0, 0);

tCanvasWidget g_sLightValCanvas2 = CanvasStruct( &g_sCanvas2, 0, 0,
                                            &g_sKentec320x240x16_SSD2119,
                                            0, 50, 28, 60,                                      // x, y, w, h
                                            CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_OPAQUE,
                                            ClrBlack, 0, ClrWhite, &g_sFontCm12, "2000", 0, 0);

// Panel 3 - Plotting
Canvas(g_sCanvas3, g_psPanels + 3, 0, 0,
       &g_sKentec320x240x16_SSD2119,
       30, 50, 200, 140,
       CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN,
       0, ClrGray, 0, 0, 0, 0, OnIMUCanvasPaint);

tCheckBoxWidget g_sPlotModeCheckBox4 = CheckBoxStruct(g_psPanels + 3,
    NULL, 0, &g_sKentec320x240x16_SSD2119,
    30, 200, 65, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Hold",
    0, OnIMUPlotModeChange2);

tCheckBoxWidget g_sPlotModeCheckBox3 = CheckBoxStruct(g_psPanels + 3,
    &g_sPlotModeCheckBox4, 0, &g_sKentec320x240x16_SSD2119,
    135, 200, 95, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Show Raw",
    0, OnIMUPlotModeChange);

// Panel 4 - Plotting
Canvas(g_sCanvas4, g_psPanels + 4, 0, 0,
       &g_sKentec320x240x16_SSD2119,
       30, 50, 200, 140,
       CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN,
       0, ClrGray, 0, 0, 0, 0, OnCanvasPaint);

tCheckBoxWidget g_sPlotModeCheckBox5 = CheckBoxStruct(g_psPanels + 4,
    NULL, 0, &g_sKentec320x240x16_SSD2119,
    30, 200, 65, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Hold",
    0, OnPlotModeChange3);

tCheckBoxWidget g_sPlotModeCheckBox6 = CheckBoxStruct(g_psPanels + 4,
    &g_sPlotModeCheckBox5, 0, &g_sKentec320x240x16_SSD2119,
    135, 200, 95, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Show Raw",
    0, OnPlotModeChange4);

// Panel 5 - Plotting
Canvas(g_sCanvas5, g_psPanels + 5, 0, 0,     // ← no static child
       &g_sKentec320x240x16_SSD2119,
       30, 50, 200, 140,
       CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN,
       0, ClrGray, 0, 0, 0, 0, OnCanvasPaint);

tCheckBoxWidget g_sPlotModeCheckBox7 = CheckBoxStruct(g_psPanels + 5,
    NULL, 0, &g_sKentec320x240x16_SSD2119,
    30, 200, 65, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Hold",
    0, OnPlotModeChange5);

tCheckBoxWidget g_sPlotModeCheckBox8 = CheckBoxStruct(g_psPanels + 5,
    &g_sPlotModeCheckBox7, 0, &g_sKentec320x240x16_SSD2119,
    135, 200, 95, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Show Raw",
    0, OnPlotModeChange6);

// Main Panels Array
tCanvasWidget g_psPanels[6] =
{
    // Panel 0: Dashboard
    CanvasStruct(0, 0, &g_sDashboard, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),

    // Panel 1: Engine / Inputs
    CanvasStruct(0, 0, &g_sEngineCanvas, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),

    // Panel 2: Light / Plotting
    CanvasStruct(0, 0, &g_sCanvas2, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),

    // Panel 3: Accelerometer / Plotting
    CanvasStruct(0, 0, &g_sCanvas3, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),

    // Panel 3: Range / Plotting 2
    CanvasStruct(0, 0, &g_sCanvas4, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
    // Panel 4: Temp humid / Plotting
    CanvasStruct(0, 0, &g_sCanvas5, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
};

Canvas(g_sDashboard, g_psPanels, &g_sDashboardSlider, &g_sFaultPanel,
       &g_sKentec320x240x16_SSD2119, 0, 0,
       320, 240, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnDashboardPaint);

// ====================== TIME CANVAS ON DASHBOARD ======================
tCanvasWidget g_sTempCanvas = CanvasStruct( &g_sDashboard, 0, 0,
                                            &g_sKentec320x240x16_SSD2119,
                                            210, 8, 60, 28,                                      // x, y, w, h
                                            CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_OPAQUE,
                                            ClrBlack, 0, ClrRed, &g_sFontCm16, "--,-- C", 0, 0);

// ====================== TIME CANVAS ON DASHBOARD ======================
tCanvasWidget g_sTimeCanvas = CanvasStruct( &g_sDashboard, 0, 0,
                                            &g_sKentec320x240x16_SSD2119,
                                            15, 8, 190, 28,                                      // x, y, w, h
                                            CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_OPAQUE,
                                            ClrBlack, 0, ClrWhite, &g_sFontCm16, "00/00/00 00:00:00", 0, 0);


                              
// ====================== MOTOR STATE TEXTBOX ======================
tCanvasWidget g_sMotorStateCanvas = CanvasStruct(&g_sDashboard, 0, 0,
                                                &g_sKentec320x240x16_SSD2119,
                                                150, 40, 80, 30,
                                                CANVAS_STYLE_OUTLINE |CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_OPAQUE,
                                                ClrDarkGreen, 0, ClrWhite, &g_sFontCm12, "IDLE", 0, 0);

// ====================== FAULTS / WARNINGS PANEL ======================
tCanvasWidget g_sFaultPanel = CanvasStruct(&g_sDashboard, &g_sMotorStateCanvas, 0,
                                            &g_sKentec320x240x16_SSD2119,
                                            45, 180, 180, 200,
                                            CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN,
                                            ClrBlack, ClrRed, ClrWhite, &g_sFontCm16, "", 0, OnFaultPanelPaint);

// ====================== DEDICATED CANVAS FOR ACK FAULT BUTTON ======================
tCanvasWidget g_sFaultButtonCanvas = CanvasStruct(&g_sDashboard, 0, 0,
                                                 &g_sKentec320x240x16_SSD2119,
                                                 198, 160, 30, 20,          // x, y, width, height
                                                 CANVAS_STYLE_FILL,           // simple black background
                                                 ClrBlue, 0, 0, 0, 0, 0, 0); // no text, no outline        

// ====================== ACK FAULT BUTTON (now inside its own canvas) ======================
tPushButtonWidget g_sClearFaultButton = RectangularButtonStruct(&g_sFaultButtonCanvas, 0, 0,
                                        &g_sKentec320x240x16_SSD2119, 198, 160, 30, 20,     // relative coords inside canvas
                                        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
                                        ClrRed, ClrBlack, ClrWhite, ClrWhite,
                                        &g_sFontCm12, "XF", 0, 0, 0, 0, OnClearFaultButtonPress);

// ====================== DASHBOARD SLIDER ======================
tSliderWidget g_sDashboardSlider = SliderStruct(&g_sDashboard, 
                                                NULL,                           // Next sibling (will be set by parent)
                                                0,                              // No children
                                                &g_sKentec320x240x16_SSD2119,
                                                10, 50, 25, 140,               // x, y, width, height  ← adjust as needed
                                                0, 100, 0,                     // min, max, initial value
                                                (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                                                SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT | SL_STYLE_VERTICAL),
                                                ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                                                &g_sFontCm16, "0", 0, 0, OnDashboardSliderChange);

// Square START button on Dashboard
tPushButtonWidget g_sStartButton = RectangularButtonStruct(&g_sDashboard, 0, 0,
                                    &g_sKentec320x240x16_SSD2119, 85, 100, 80, 45,     // square-ish 80x55
                                    PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
                                    ClrRed, ClrBlack, ClrWhite, ClrWhite,
                                    &g_sFontCm22, "START", 0, 0, 0, 0, OnStartButtonPress);



// ====================== PERMANENT SIDE BUTTONS ======================
// These are always on screen, on every panel

tPushButtonWidget g_psSideButtons[6] =
{
    // 0 - Engine
    RectangularButtonStruct(0, 0, 0,
        &g_sKentec320x240x16_SSD2119, 235, 40, 75, 25,
        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
        ClrDarkBlue, ClrBlack, ClrWhite, ClrWhite,
        &g_sFontCm12b, "Dash", 0, 0, 0, 0, OnDashButtonPress),

    // 1 - Light
    RectangularButtonStruct(0, 0, 0,
        &g_sKentec320x240x16_SSD2119, 235, 70, 75, 25,
        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
        ClrDarkBlue, ClrBlack, ClrWhite, ClrWhite,
        &g_sFontCm12b, "Settings", 0, 0, 0, 0, OnDashButtonPress),

    // 2 - Temp
    RectangularButtonStruct(0, 0, 0,
        &g_sKentec320x240x16_SSD2119, 235, 100, 75, 25,
        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
        ClrDarkBlue, ClrBlack, ClrWhite, ClrWhite,
        &g_sFontCm12b, "Light", 0, 0, 0, 0, OnDashButtonPress),
    // 3 - Accelerometer
    RectangularButtonStruct(0, 0, 0,
        &g_sKentec320x240x16_SSD2119, 235, 130, 75, 25,
        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
        ClrDarkBlue, ClrBlack, ClrWhite, ClrWhite,
        &g_sFontCm12b, "Accel", 0, 0, 0, 0, OnDashButtonPress),
    // 3 - Range
    RectangularButtonStruct(0, 0, 0,
        &g_sKentec320x240x16_SSD2119, 235, 160, 75, 25,
        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
        ClrDarkBlue, ClrBlack, ClrWhite, ClrWhite,
        &g_sFontCm12b, "Range", 0, 0, 0, 0, OnDashButtonPress),
    // temp /humidity
    RectangularButtonStruct(0, 0, 0,
        &g_sKentec320x240x16_SSD2119, 235, 190, 75, 25,
        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
        ClrDarkBlue, ClrBlack, ClrWhite, ClrWhite,
        &g_sFontCm12b, "Temp", 0, 0, 0, 0, OnDashButtonPress)
};

// Plot callbacks
void OnOPTPlotModeChange(tWidget *psWidget, uint32_t bSelected)
{
    g_eOPTPlotMode = bSelected ? OPT_PLOT_MODE_RAW_AND_FILTERED : OPT_PLOT_MODE_FILTERED_ONLY;    
    if (xEventGroup) 
        xEventGroupSetBits(xEventGroup, EVENT_BTN_TOGGLE_PLOT);
    if (g_ui32Panel == 2) 
        WidgetPaint((tWidget *)&g_sCanvas2);
}

void OnOPTPlotModeChange2(tWidget *psWidget, uint32_t bSelected)
{
    g_bOPTDisplayHold = bSelected;
    //if (xEventGroup) xEventGroupSetBits(xEventGroup, EVENT_DISPLAY_HOLD);
    if (g_ui32Panel == 2) 
        WidgetPaint((tWidget *)&g_sCanvas2);
}


// Plot callbacks
void OnIMUPlotModeChange(tWidget *psWidget, uint32_t bSelected)
{
    g_eIMUPlotMode = bSelected ? IMU_PLOT_MODE_RAW_AND_FILTERED : IMU_PLOT_MODE_FILTERED_ONLY;
    //if (xEventGroup) xEventGroupSetBits(xEventGroup, EVENT_BTN_TOGGLE_PLOT);
    if (g_ui32Panel == 3) 
        WidgetPaint((tWidget *)&g_sCanvas3);
}

void OnIMUPlotModeChange2(tWidget *psWidget, uint32_t bSelected)
{
    g_bIMUDisplayHold = bSelected;
    //if (xEventGroup) xEventGroupSetBits(xEventGroup, EVENT_DISPLAY_HOLD);
    if (g_ui32Panel == 3) 
        WidgetPaint((tWidget *)&g_sCanvas3);
}

// Plot callbacks
void OnPlotModeChange3(tWidget *psWidget, uint32_t bSelected)
{
    g_ePlotMode = bSelected ? IMU_PLOT_MODE_RAW_AND_FILTERED : IMU_PLOT_MODE_FILTERED_ONLY;
    // if (xEventGroup) xEventGroupSetBits(xEventGroup, EVENT_BTN_TOGGLE_PLOT);
    if (g_ui32Panel == 4) 
        WidgetPaint((tWidget *)&g_sCanvas4);
}

void OnPlotModeChange4(tWidget *psWidget, uint32_t bSelected)
{
    g_bDisplayHold = bSelected;
    // if (xEventGroup) xEventGroupSetBits(xEventGroup, EVENT_DISPLAY_HOLD);
    if (g_ui32Panel == 4) 
        WidgetPaint((tWidget *)&g_sCanvas4);
}

// Plot callbacks // temp 
void OnPlotModeChange5(tWidget *psWidget, uint32_t bSelected)
{
    g_ePlotMode = bSelected ? IMU_PLOT_MODE_RAW_AND_FILTERED : IMU_PLOT_MODE_FILTERED_ONLY;
    // if (xEventGroup) xEventGroupSetBits(xEventGroup, EVENT_BTN_TOGGLE_PLOT);
    if (g_ui32Panel == 5) 
        WidgetPaint((tWidget *)&g_sCanvas5);
}

void OnPlotModeChange6(tWidget *psWidget, uint32_t bSelected)
{
    g_bDisplayHold = bSelected;
    //if (xEventGroup) xEventGroupSetBits(xEventGroup, EVENT_DISPLAY_HOLD);
    if (g_ui32Panel == 5) 
        WidgetPaint((tWidget *)&g_sCanvas5);
}

void OnIMUCanvasPaint(tWidget *psWidget, tContext *psContext)
{
    uint32_t ui32Idx;
    tRectangle sRect;
    int32_t gx1, gy1, gx2, gy2, gy1f, gy2f;    
    const uint32_t yMax = 9000;
    const uint32_t canvas_xMin = 30, canvas_xRange = 210;
    const uint32_t canvas_yMin = 50, canvas_yRange = 140;
    // static char buff1[4];
    // static char buff2[4];
    // Background
    GrContextForegroundSet(psContext, ClrBlack);
    sRect.i16XMin = canvas_xMin+2; 
    sRect.i16YMin = canvas_yMin+2;
    sRect.i16XMax = canvas_xMin-2 + canvas_xRange;
    sRect.i16YMax = canvas_yMin-2 + canvas_yRange;
    GrRectFill(psContext, &sRect);

    // Grid
    GrContextForegroundSet(psContext, ClrGray);
    sRect.i16XMin = canvas_xMin; sRect.i16YMin = canvas_yMin;
    sRect.i16XMax = canvas_xMin + canvas_xRange;
    sRect.i16YMax = canvas_yMin + canvas_yRange;
    GrRectDraw(psContext, &sRect);

    // Labels
    GrContextFontSet(psContext, &g_sFontCm16);
    GrContextForegroundSet(psContext, ClrSilver);
    GrStringDrawCentered(psContext, "2000", -1, 10, 33, 0);
    GrStringDrawCentered(psContext, "10",   -1, 10, 160, 0);
    GrStringDrawCentered(psContext, "Now",  -1, 30, 175, 0);
    GrStringDrawCentered(psContext, "-10s", -1, 210, 175, 0);

    static char modeStr[15];
     
    if(g_eIMUPlotMode == IMU_PLOT_MODE_RAW_AND_FILTERED)
    {
        usprintf(modeStr, "Raw + Filtered"); 
    }
    else if(g_eIMUPlotMode == IMU_PLOT_MODE_FILTERED_ONLY) 
    {
        usprintf(modeStr, "Filtered Only");     
    }

    GrContextForegroundSet(psContext, ClrWhite);
    GrStringDrawCentered(psContext, modeStr, -1, canvas_xMin + canvas_xRange/2, 8, 0);

    // Draw plot lines
    for (ui32Idx = 0; ui32Idx < PLOT_HISTORY_SIZE - 1; ui32Idx++)
    {
        uint32_t dataIdx1 = (g_plotIMUIndex + ui32Idx) % PLOT_HISTORY_SIZE;
        uint32_t dataIdx2 = (g_plotIMUIndex + ui32Idx + 1) % PLOT_HISTORY_SIZE;

        uint32_t x1  = g_plotIMUxHistory[dataIdx1];
        uint32_t x2  = g_plotIMUxHistory[dataIdx2];

        uint32_t y1 = g_plotIMUyHistory[dataIdx1];
        uint32_t y2 = g_plotIMUyHistory[dataIdx2];

        uint32_t mag1 = g_plotIMUmagHistory[dataIdx1];
        uint32_t mag2 = g_plotIMUmagHistory[dataIdx2];

        float sX1  = (float)x1  / yMax; if (sX1  > 1.0f) sX1  = 1.0f;
        float sX2  = (float)x2  / yMax; if (sX2  > 1.0f) sX2  = 1.0f;

        float sY1 = (float)y1 / yMax; if (sY1 > 1.0f) sY1 = 1.0f;
        float sY2 = (float)y2 / yMax; if (sY2 > 1.0f) sY2 = 1.0f;

        gy1  = canvas_yMin + canvas_yRange - (int32_t)(sX1  * canvas_yRange);
        gy2  = canvas_yMin + canvas_yRange - (int32_t)(sX2  * canvas_yRange);
        gy1f = canvas_yMin + canvas_yRange - (int32_t)(sY1 * canvas_yRange);
        gy2f = canvas_yMin + canvas_yRange - (int32_t)(sY2 * canvas_yRange);

        gx1 = canvas_xMin + (ui32Idx * canvas_xRange / (PLOT_HISTORY_SIZE - 1));
        gx2 = canvas_xMin + ((ui32Idx + 1) * canvas_xRange / (PLOT_HISTORY_SIZE - 1));
        
        if (g_eIMUPlotMode == IMU_PLOT_MODE_RAW_AND_FILTERED)
        {   
            // usprintf(buff2, "%3d",filt1); 
            // CanvasTextSet(&g_sLightValCanvas3, buff2);
            // WidgetPaint((tWidget *)&g_sLightValCanvas3);
            GrContextForegroundSet(psContext, ClrCyan);            
            GrLineDraw(psContext, gx1, gy1-3, gx2, gy2-3);
        }

        // GrContextFontSet(psContext, &g_sFontCm12);
        // GrContextForegroundSet(psContext, ClrYellow);
        // usprintf(buff,"%3d",(int32_t)filt1);
        // GrStringDrawCentered(psContext, buff, -1, 200, 65, 1);

        // usprintf(buff1, "%4d",raw1); 
        // CanvasTextSet(&g_sLightValCanvas, buff1);
        // WidgetPaint((tWidget *)&g_sLightValCanvas);
        GrContextForegroundSet(psContext, ClrYellow);
        GrLineDraw(psContext, gx1, gy1f-3, gx2, gy2f-3);
    }

}

void OnCanvasPaint(tWidget *psWidget, tContext *psContext)
{
    // uint32_t ui32Idx;
    // tRectangle sRect;
    // int32_t x1, y1, x2, y2, y1f, y2f;    
    // const uint32_t yMax = 2000;
    // const uint32_t canvas_xMin = 30, canvas_xRange = 210;
    // const uint32_t canvas_yMin = 50, canvas_yRange = 140;
    // // static char buff1[4];
    // // static char buff2[4];
    // // Background
    // GrContextForegroundSet(psContext, ClrBlack);
    // sRect.i16XMin = canvas_xMin+2; 
    // sRect.i16YMin = canvas_yMin+2;
    // sRect.i16XMax = canvas_xMin-2 + canvas_xRange;
    // sRect.i16YMax = canvas_yMin-2 + canvas_yRange;
    // GrRectFill(psContext, &sRect);

    // // Grid
    // GrContextForegroundSet(psContext, ClrGray);
    // sRect.i16XMin = canvas_xMin; sRect.i16YMin = canvas_yMin;
    // sRect.i16XMax = canvas_xMin + canvas_xRange;
    // sRect.i16YMax = canvas_yMin + canvas_yRange;
    // GrRectDraw(psContext, &sRect);

    // // Labels
    // GrContextFontSet(psContext, &g_sFontCm16);
    // GrContextForegroundSet(psContext, ClrSilver);
    // GrStringDrawCentered(psContext, "2000", -1, 10, 33, 0);
    // GrStringDrawCentered(psContext, "10",   -1, 10, 160, 0);
    // GrStringDrawCentered(psContext, "Now",  -1, 30, 175, 0);
    // GrStringDrawCentered(psContext, "-10s", -1, 210, 175, 0);

    // static char modeStr[15];
     
    // if(g_eIMUPlotMode == IMU_PLOT_MODE_RAW_AND_FILTERED)
    // {
    //     usprintf(modeStr, "Raw + Filtered"); 
    // }
    // else if(g_eIMUPlotMode == IMU_PLOT_MODE_FILTERED_ONLY) 
    // {
    //     usprintf(modeStr, "Filtered Only");     
    // }

    // GrContextForegroundSet(psContext, ClrWhite);
    // GrStringDrawCentered(psContext, modeStr, -1, canvas_xMin + canvas_xRange/2, 8, 0);

    // // Draw plot lines
    // for (ui32Idx = 0; ui32Idx < PLOT_HISTORY_SIZE - 1; ui32Idx++)
    // {
    //     uint32_t dataIdx1 = (g_plotIMUIndex + ui32Idx) % PLOT_HISTORY_SIZE;
    //     uint32_t dataIdx2 = (g_plotIMUIndex + ui32Idx + 1) % PLOT_HISTORY_SIZE;

    //     uint32_t raw1  = g_plotIMURawHistory[dataIdx1];
    //     uint32_t raw2  = g_plotIMURawHistory[dataIdx2];
    //     uint32_t filt1 = g_plotIMUFiltHistory[dataIdx1];
    //     uint32_t filt2 = g_plotIMUFiltHistory[dataIdx2];

    //     float sRaw1  = (float)raw1  / yMax; if (sRaw1  > 1.0f) sRaw1  = 1.0f;
    //     float sRaw2  = (float)raw2  / yMax; if (sRaw2  > 1.0f) sRaw2  = 1.0f;
    //     float sFilt1 = (float)filt1 / yMax; if (sFilt1 > 1.0f) sFilt1 = 1.0f;
    //     float sFilt2 = (float)filt2 / yMax; if (sFilt2 > 1.0f) sFilt2 = 1.0f;

    //     y1  = canvas_yMin + canvas_yRange - (int32_t)(sRaw1  * canvas_yRange);
    //     y2  = canvas_yMin + canvas_yRange - (int32_t)(sRaw2  * canvas_yRange);
    //     y1f = canvas_yMin + canvas_yRange - (int32_t)(sFilt1 * canvas_yRange);
    //     y2f = canvas_yMin + canvas_yRange - (int32_t)(sFilt2 * canvas_yRange);

    //     x1 = canvas_xMin + (ui32Idx * canvas_xRange / (PLOT_HISTORY_SIZE - 1));
    //     x2 = canvas_xMin + ((ui32Idx + 1) * canvas_xRange / (PLOT_HISTORY_SIZE - 1));
        
    //     if (g_eIMUPlotMode == IMU_PLOT_MODE_RAW_AND_FILTERED)
    //     {   
    //         // usprintf(buff2, "%3d",filt1); 
    //         // CanvasTextSet(&g_sLightValCanvas3, buff2);
    //         // WidgetPaint((tWidget *)&g_sLightValCanvas3);
    //         GrContextForegroundSet(psContext, ClrCyan);
    //         GrLineDraw(psContext, x1, y1-3, x2, y2-3);
    //     }

    //     // GrContextFontSet(psContext, &g_sFontCm12);
    //     // GrContextForegroundSet(psContext, ClrYellow);
    //     // usprintf(buff,"%3d",(int32_t)filt1);
    //     // GrStringDrawCentered(psContext, buff, -1, 200, 65, 1);

    //     // usprintf(buff1, "%4d",raw1); 
    //     // CanvasTextSet(&g_sLightValCanvas, buff1);
    //     // WidgetPaint((tWidget *)&g_sLightValCanvas);
    //     GrContextForegroundSet(psContext, ClrYellow);
    //     GrLineDraw(psContext, x1, y1f-3, x2, y2f-3);
    // }
}
void OnOPTCanvasPaint(tWidget *psWidget, tContext *psContext)
{
    uint32_t ui32Idx;
    tRectangle sRect;
    int32_t x1, y1, x2, y2, y1f, y2f;    
    const uint32_t yMax = 2000;
    const uint32_t canvas_xMin = 30, canvas_xRange = 210;
    const uint32_t canvas_yMin = 50, canvas_yRange = 140;
    static char buff1[4];
    static char buff2[4];
    // Background
    GrContextForegroundSet(psContext, ClrBlack);
    sRect.i16XMin = canvas_xMin+2; 
    sRect.i16YMin = canvas_yMin+2;
    sRect.i16XMax = canvas_xMin-2 + canvas_xRange;
    sRect.i16YMax = canvas_yMin-2 + canvas_yRange;
    GrRectFill(psContext, &sRect);

    // Grid
    GrContextForegroundSet(psContext, ClrGray);
    sRect.i16XMin = canvas_xMin; sRect.i16YMin = canvas_yMin;
    sRect.i16XMax = canvas_xMin + canvas_xRange;
    sRect.i16YMax = canvas_yMin + canvas_yRange;
    GrRectDraw(psContext, &sRect);

    // Labels
    GrContextFontSet(psContext, &g_sFontCm16);
    GrContextForegroundSet(psContext, ClrSilver);
    GrStringDrawCentered(psContext, "2000", -1, 10, 33, 0);
    GrStringDrawCentered(psContext, "10",   -1, 10, 160, 0);
    GrStringDrawCentered(psContext, "Now",  -1, 30, 175, 0);
    GrStringDrawCentered(psContext, "-10s", -1, 210, 175, 0);

    static char modeStr[15];
     
    if(g_eOPTPlotMode == OPT_PLOT_MODE_RAW_AND_FILTERED)
    {
        usprintf(modeStr, "Raw + Filtered"); 
    }
    else if(g_eOPTPlotMode == OPT_PLOT_MODE_FILTERED_ONLY) 
    {
        usprintf(modeStr, "Filtered Only");     
    }
           
    GrContextForegroundSet(psContext, ClrWhite);
    GrStringDrawCentered(psContext, modeStr, -1, canvas_xMin + canvas_xRange/2, 8, 0);

    // Draw plot lines
    for (ui32Idx = 0; ui32Idx < PLOT_HISTORY_SIZE - 1; ui32Idx++)
    {
        uint32_t dataIdx1 = (g_plotOPTIndex + ui32Idx) % PLOT_HISTORY_SIZE;
        uint32_t dataIdx2 = (g_plotOPTIndex + ui32Idx + 1) % PLOT_HISTORY_SIZE;

        uint32_t raw1  = g_plotOPTRawHistory[dataIdx1];
        uint32_t raw2  = g_plotOPTRawHistory[dataIdx2];
        uint32_t filt1 = g_plotOPTFiltHistory[dataIdx1];
        uint32_t filt2 = g_plotOPTFiltHistory[dataIdx2];

        float sRaw1  = (float)raw1  / yMax; if (sRaw1  > 1.0f) sRaw1  = 1.0f;
        float sRaw2  = (float)raw2  / yMax; if (sRaw2  > 1.0f) sRaw2  = 1.0f;
        float sFilt1 = (float)filt1 / yMax; if (sFilt1 > 1.0f) sFilt1 = 1.0f;
        float sFilt2 = (float)filt2 / yMax; if (sFilt2 > 1.0f) sFilt2 = 1.0f;

        y1  = canvas_yMin + canvas_yRange - (int32_t)(sRaw1  * canvas_yRange);
        y2  = canvas_yMin + canvas_yRange - (int32_t)(sRaw2  * canvas_yRange);
        y1f = canvas_yMin + canvas_yRange - (int32_t)(sFilt1 * canvas_yRange);
        y2f = canvas_yMin + canvas_yRange - (int32_t)(sFilt2 * canvas_yRange);

        x1 = canvas_xMin + (ui32Idx * canvas_xRange / (PLOT_HISTORY_SIZE - 1));
        x2 = canvas_xMin + ((ui32Idx + 1) * canvas_xRange / (PLOT_HISTORY_SIZE - 1));
        
        if (g_eOPTPlotMode == OPT_PLOT_MODE_RAW_AND_FILTERED)
        {   
            usprintf(buff2, "%4d",filt1); 
            CanvasTextSet(&g_sLightValCanvas2, buff2);
            WidgetPaint((tWidget *)&g_sLightValCanvas2);
            GrContextForegroundSet(psContext, ClrCyan);            
            GrLineDraw(psContext, x1, y1-3, x2, y2-3);
        }
        usprintf(buff1, "%4d",raw1); 
        CanvasTextSet(&g_sLightValCanvas, buff1);
        WidgetPaint((tWidget *)&g_sLightValCanvas);
        GrContextForegroundSet(psContext, ClrYellow);
        GrLineDraw(psContext, x1, y1f-3, x2, y2f-3);
    }
}

// Global value for the speed / gauge
// Temp hooks for later
//uint32_t g_ui32Speed = 80;        
//uint32_t g_ui32Temp = 20;

void OnDashboardPaint(tWidget *psWidget, tContext *psContext)
{
    //tRectangle sRect;
    char speedtext[16];
    //char timetext[16];
    

    //GrImageDraw(psContext, g_pui8StartBtnImage, 110, 70);

    // Background
    // GrContextForegroundSet(psContext, ClrBlack);
    // sRect.i16XMin = 0; sRect.i16YMin = 0;
    // sRect.i16XMax = 319; sRect.i16YMax = 165;
    // GrRectFill(psContext, &sRect);

    // Top bar
    // GrContextForegroundSet(psContext, ClrDarkBlue);
    // sRect.i16YMax = 28;
    // GrRectFill(psContext, &sRect);
    



    // Big Speed Number
    GrContextFontSet(psContext, &g_sFontCm34);
    GrContextForegroundSet(psContext, ClrCyan);
    usprintf(speedtext, "%d", g_ui32Speed);
    GrStringDrawCentered(psContext, speedtext, -1, 60, 75, 1);

    GrContextFontSet(psContext, &g_sFontCm16);
    GrStringDraw(psContext, "Km/H", -1, 85, 75, 0);

    UpdateMotorStateCanvas();
    UpdateFaultPanel();
    UpdateTemp(psContext);
    
    // Ensure slider is painted
    WidgetPaint((tWidget *)&g_sDashboardSlider);
    
}

void OnDashButtonPress(tWidget *psWidget)
{
    uint32_t targetPanel = g_ui32Panel;

    for (uint32_t i = 0; i < 6; i++)
    {
        if (psWidget == (tWidget *)&g_psSideButtons[i])
        {
            targetPanel = i;        // Direct mapping: button index = panel index
            break;
        }
    }

    if (targetPanel != g_ui32Panel)
    {
        WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));
        g_ui32Panel = targetPanel;
        WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
        WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));
    }

    // Highlight active button
    for (uint32_t i = 0; i < 6; i++)
    {
        PushButtonFillColorSet(&g_psSideButtons[i],
            (i == g_ui32Panel) ? ClrBlue : ClrDarkBlue);
        WidgetPaint((tWidget *)&g_psSideButtons[i]);
    }
}

void OnStartButtonPress(tWidget *psWidget)
{
    SafeUARTprintf("START button pressed!\r\n");
    if (g_eMotorState == MOTOR_IDLE) 
    {
        g_startDisplay = false;
        SetMotorState(MOTOR_STARTING);
        // Hide the START button immediately
        //WidgetRemove((tWidget *)&g_sStartButton);
        //WidgetPaint((tWidget *)&g_sDashboard);
    }
    // else
    // {
    //     //WidgetRemove((tWidget *)&g_sStartButton);
    // }
}

// ====================== TIME UPDATING ======================

void UpdateTime(void)
{
    static uint8_t lastSeconds = 0xFF;

    if (g_ui8Seconds == lastSeconds)
        return;

    lastSeconds = g_ui8Seconds;
    static char buffer[17];
    
    usprintf(buffer, "%02d/%02d/%02d %02d:%02d:%02d",
             g_ui8Day, g_ui8Month, (int)(g_ui16Year % 100),
             g_ui8Hours, g_ui8Minutes, g_ui8Seconds);
             

    CanvasTextSet(&g_sTimeCanvas, buffer);
    WidgetPaint((tWidget *)&g_sTimeCanvas);
}

void UpdateTemp(tContext *psContext)
{
    static char temptext[16];

    // Dashboard temperature now comes from SHT31 instead of dummy g_ui32Temp
    if (g_bSHT31Valid)
    {
        FormatSignedX100(temptext, "", g_i32CabinTempCx100, "C");
    }
    else
    {
        usprintf(temptext, "--.-- C");
    }

    GrContextFontSet(psContext, &g_sFontCm20);

    if (!g_bSHT31Valid)
    {
        GrContextForegroundSet(psContext, ClrRed);
    }
    else if (g_bCoolingOn)
    {
        GrContextForegroundSet(psContext, ClrCyan);
    }
    else if (g_bHeatingOn)
    {
        GrContextForegroundSet(psContext, ClrOrange);
    }
    else
    {
        GrContextForegroundSet(psContext, ClrWhite);
    }

    //GrStringDrawCentered(psContext, temptext, -1, 240, 20, 0);

    // Reset colour after drawing temp
    GrContextForegroundSet(psContext, ClrWhite);

    CanvasTextSet(&g_sTempCanvas, temptext);
    WidgetPaint((tWidget *)&g_sTempCanvas);
}

void OnFaultPanelPaint(tWidget *psWidget, tContext *psContext)
{
    //tRectangle sRect;    

    GrContextFontSet(psContext, &g_sFontCm12);
    GrContextForegroundSet(psContext, ClrWhite);
    //45, 160, 180, 200,
    // sRect.i16XMin = 45; sRect.i16YMin = 160;
    // sRect.i16XMax = 180; sRect.i16YMax = 255;
    // GrContextForegroundSet(psContext, ClrBlack);
    // GrRectFill(psContext, &sRect);
    // GrContextForegroundSet(psContext, ClrRed);
    // GrRectDraw(psContext, &sRect);

    GrStringDraw(psContext, "FAULTS & SAFETY", -1, 50, 185, 0);

    // Faults
    GrContextForegroundSet(psContext, (g_ui32ActiveFaults & FAULT_EBRAKE) ? ClrRed : ClrDarkGray);
    GrStringDraw(psContext, "EBRAKE", -1, 50, 200, 0);

    GrContextForegroundSet(psContext, (g_ui32ActiveFaults & FAULT_OVERCURRENT) ? ClrRed : ClrDarkGray);
    GrStringDraw(psContext, "OCURR", -1, 120, 200, 0);

    GrContextForegroundSet(psContext, (g_ui32ActiveFaults & FAULT_OVERTEMP) ? ClrRed : ClrDarkGray);
    GrStringDraw(psContext, "OTEMP", -1, 50, 220, 0);

    GrContextForegroundSet(psContext, (g_ui32ActiveFaults & FAULT_SENSOR) ? ClrRed : ClrDarkGray);
    GrStringDraw(psContext, "SENS", -1, 120, 220, 0);

    // Warnings
    GrContextForegroundSet(psContext, ClrYellow);
    if (g_ui32ActiveWarnings & WARNING_LOW_VOLTAGE)
        GrStringDraw(psContext, "LOW VOLT", -1, 50, 220, 0);
    if (g_ui32ActiveWarnings & WARNING_HIGH_TEMP)
        GrStringDraw(psContext, "HI TEMP", -1, 120, 220, 0);

    if (g_ui32ActiveSafety)
    {
        GrContextForegroundSet(psContext, ClrOrange);
        GrStringDraw(psContext, "SAFETY ACTIVE", -1, 210, 242, 0);
    }
}

void UpdateMotorStateCanvas(void)
{
    uint32_t fillColor;
    const char* motorStr;

    switch (g_eMotorState)
    {
        case MOTOR_RUNNING:
            fillColor = ClrGreen;           // Normal operation
            motorStr = "RUNNING";
            break;

        case MOTOR_FAULT:
            fillColor = ClrRed;             // Fault / E-stop
            motorStr = "FAULT";
            break;

        case MOTOR_IDLE:
        case MOTOR_STARTING:
        default:
            fillColor = ClrOrange;          // Stopped / Starting
            motorStr = (g_eMotorState == MOTOR_IDLE) ? "IDLE" : "STARTING";
            break;
    }

    // Update text and background color
    CanvasTextSet(&g_sMotorStateCanvas, motorStr);
    CanvasFillColorSet(&g_sMotorStateCanvas, fillColor);
    
    // Optional: Make text bold/contrasting
    if (g_eMotorState == MOTOR_FAULT)
        CanvasTextColorSet(&g_sMotorStateCanvas, ClrWhite);
    else
        CanvasTextColorSet(&g_sMotorStateCanvas, ClrBlack);

    WidgetPaint((tWidget *)&g_sMotorStateCanvas);
}

// Optional: Force full dashboard repaint (use sparingly)
void RefreshDashboard(void)
{
    WidgetPaint((tWidget *)&g_sDashboard);
}

void SetMotorState(MotorState_t newState)
{
    if (newState == g_eMotorState)
        return;

    g_eMotorState = newState;
    

    if (xEventGroup)
        xEventGroupSetBits(xEventGroup, EVENT_MOTOR_STATE_CHANGE);

    if (g_ui32Panel == 0)
    {
        UpdateMotorStateCanvas();
        UpdateFaultPanel();
    }
        
}

void OnDashboardSliderChange(tWidget *psWidget, int32_t i32Value)
{
    static char pcSliderText[6];

    if (psWidget == (tWidget *)&g_sDashboardSlider)
    {
        usprintf(pcSliderText, "%3d", (int)i32Value);
        SliderTextSet(&g_sDashboardSlider, pcSliderText);

        // Example: Link to motor speed
        g_ui32Speed = (uint32_t)i32Value;   // 0-200 km/h example

        // Refresh only the dashboard (safer than full repaint)
        if (g_ui32Panel == 0)
            WidgetPaint((tWidget *)&g_sDashboard);
    }
}

// ====================== SLIDER CALLBACK ======================

void OnEngineCanvasPaint(tWidget *psWidget, tContext *psContext)
{
    tRectangle sRect;
    char buf[32];

    // Background
    GrContextForegroundSet(psContext, ClrBlack);
    sRect.i16XMin = 0; sRect.i16YMin = 24;
    sRect.i16XMax = 240; sRect.i16YMax = 260;
    GrRectFill(psContext, &sRect);

    GrContextFontSet(psContext, &g_sFontCm16);
    GrContextForegroundSet(psContext, ClrWhite);

    // Changed GUI element positions to prevent overlap
    usprintf(buf, "Max Engine Accel: %3d", g_ui32AccelMaxVal);
    GrStringDraw(psContext, buf, -1, 40, 40, 0);

    usprintf(buf, "Range Tr Dist: %3d cm", g_ui32DetRangeVal);
    GrStringDraw(psContext, buf, -1, 40, 80, 0);

    usprintf(buf, "Crash Threshold: %3d", g_ui32CrashThreshold);
    GrStringDraw(psContext, buf, -1, 40, 120, 0);

    usprintf(buf, "Min Lux Threshold: %3d", g_ui32LuxMinThreshold);
    GrStringDraw(psContext, buf, -1, 40, 160, 0);

    // SHT31 values are placed below existing controls
    // prevents humidity text overlapping with Min Lux Threshold
    if (g_bSHT31Valid)
    {
        GrContextForegroundSet(psContext, ClrWhite);

        FormatSignedX100(buf, "Cabin Temp: ", g_i32CabinTempCx100, "C");
        GrStringDraw(psContext, buf, -1, 40, 180, 0);

        FormatUnsignedX100(buf, "Humidity: ", g_ui32CabinHumidityRHx100, "%RH");
        GrStringDraw(psContext, buf, -1, 40, 200, 0);

        if (g_bCoolingOn)
        {
            GrContextForegroundSet(psContext, ClrCyan);
            GrStringDraw(psContext, "Climate: Cooling", -1, 40, 210, 0);
        }
        else if (g_bHeatingOn)
        {
            GrContextForegroundSet(psContext, ClrOrange);
            GrStringDraw(psContext, "Climate: Heating", -1, 40, 210, 0);
        }
        else
        {
            GrContextForegroundSet(psContext, ClrGreen);
            GrStringDraw(psContext, "Climate: OK", -1, 40, 210, 0);
        }
    }
    else
    {
        GrContextForegroundSet(psContext, ClrRed);
        GrStringDraw(psContext, "Climate: No Data", -1, 40, 210, 0);
    }

    GrContextForegroundSet(psContext, ClrWhite);
}

void OnCrashTolSliderChange(tWidget *psWidget, int32_t i32Value)
{
    static char pcSliderText[6];

    if (psWidget == (tWidget *)&g_sCrashTolSlider)
    {
        usprintf(pcSliderText, "%3d", (int)i32Value);
        SliderTextSet(&g_sCrashTolSlider, pcSliderText);

        // Update the variable you want to control
        g_ui32CrashThreshold = (uint32_t)i32Value * 10;   // Example scaling

        // ONLY repaint when slider actually changes
        if (g_ui32Panel == 1)
        {
            WidgetPaint((tWidget *)&g_sEngineCanvas);
        }
    }
}

void OnAccelMaxSliderChange(tWidget *psWidget, int32_t i32Value)
{
    static char pcSliderText[6];

    if (psWidget == (tWidget *)&g_sAccelMaxSlider)
    {
        usprintf(pcSliderText, "%3d", (int)i32Value);
        SliderTextSet(&g_sAccelMaxSlider, pcSliderText);

        // Update the variable you want to control
        g_ui32AccelMaxVal = (uint32_t)i32Value;   // Example scaling

        // ONLY repaint when slider actually changes
        if (g_ui32Panel == 1)
        {
            WidgetPaint((tWidget *)&g_sEngineCanvas);
        }
    }
}

void OnDetRangeSliderChange(tWidget *psWidget, int32_t i32Value)
{
    static char pcSliderText[6];

    if (psWidget == (tWidget *)&g_sDetRangeSlider)
    {
        usprintf(pcSliderText, "%3d", (int)i32Value);
        SliderTextSet(&g_sDetRangeSlider, pcSliderText);

        // Update the variable you want to control
        g_ui32DetRangeVal = (uint32_t)i32Value;   // Example scaling

        // ONLY repaint when slider actually changes
        if (g_ui32Panel == 1)
        {
            WidgetPaint((tWidget *)&g_sEngineCanvas);
        }
    }
}

void OnLuxMinSliderChange(tWidget *psWidget, int32_t i32Value)
{
    static char pcSliderText[6];

    if (psWidget == (tWidget *)&g_sLuxMinSlider)
    {
        usprintf(pcSliderText, "%3d", (int)i32Value);
        SliderTextSet(&g_sLuxMinSlider, pcSliderText);

        // Update the variable you want to control
        g_ui32LuxMinThreshold = (uint32_t)i32Value;   // Example scaling

        // ONLY repaint when slider actually changes
        if (g_ui32Panel == 1)
        {
            WidgetPaint((tWidget *)&g_sEngineCanvas);
        }
    }
}

void UpdateFaultPanel(void)
{
    // Update fault text panel
    WidgetPaint((tWidget *)&g_sFaultPanel);

    static bool lastFaultState = false;
    bool hasFault = (g_ui32ActiveFaults != 0);

    if (hasFault == lastFaultState)
        return;

    lastFaultState = hasFault;

    if (hasFault)
        ShowAckFaultButton();
    else
        HideAckFaultButton();
}

void OnClearFaultButtonPress(tWidget *psWidget)
{
    SafeUARTprintf("ACK FAULT button pressed - Clearing faults and resuming motor!\r\n");
    
    g_bFaultAcknowledged = true;
    g_ui32ActiveFaults = 0;
    g_bFaultButtonVisible = false;          // force hidden state
    
    SetMotorState(MOTOR_IDLE);
    
    // UpdateFaultPanel() will now correctly hide it
    UpdateFaultPanel();
}


void ShowAckFaultButton(void)
{
    if (!g_bFaultButtonVisible)
    {
        WidgetAdd((tWidget *)&g_sDashboard, (tWidget *)&g_sFaultButtonCanvas);
        g_bFaultButtonVisible = true;
    }
    WidgetPaint((tWidget *)&g_sFaultButtonCanvas);
}

void HideAckFaultButton(void)
{
    if (g_bFaultButtonVisible)
    {
        WidgetRemove((tWidget *)&g_sFaultButtonCanvas);
        g_bFaultButtonVisible = false;
    }
}
