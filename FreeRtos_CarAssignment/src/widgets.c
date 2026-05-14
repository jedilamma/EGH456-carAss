#include "taskman.h"
#include "widgets.h"
#include "images.h"
#include "slider.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "utils/uartstdio.h"


// ====================== Widget Definitions ======================
// ====================== PANEL 1 - ENGINE (Clean Version) ======================

// ====================== PANEL 1 - ENGINE ======================
Canvas(g_sEngineCanvas, g_psPanels + 1, 0, 0,
       &g_sKentec320x240x16_SSD2119, 0, 24, 230, 160, 
       CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN, 0, ClrGray, 
       0, 0, 0, 0, OnEngineCanvasPaint);

// ====================== ENGINE PANEL SLIDER (Vertical) ======================
tSliderWidget g_sEngineSlider = SliderStruct(&g_sEngineCanvas, 
                                             NULL,                  // Next sibling
                                             0,                     // No children
                                             &g_sKentec320x240x16_SSD2119,
                                             25, 70, 135, 20,       // x, y, width, height → Vertical
                                             0, 100, 50,            // min, max, initial value
                                             (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | 
                                              SL_STYLE_OUTLINE | SL_STYLE_TEXT | 
                                              SL_STYLE_BACKG_TEXT),
                                             ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                                             &g_sFontCm16, "50", 0, 0, OnEngineSliderChange);

// Panel 2 - Plotting
Canvas(g_sCanvas2, g_psPanels + 2, &g_sPlotModeCheckBox, 0,     // ← IMPORTANT: point to first child
       &g_sKentec320x240x16_SSD2119,
       30, 50, 200, 140,
       CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN,
       0, ClrGray, 0, 0, 0, 0, OnCanvasPaint);

tCheckBoxWidget g_sPlotModeCheckBox2 = CheckBoxStruct(g_psPanels + 2,
    NULL, 0, &g_sKentec320x240x16_SSD2119,
    30, 200, 65, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Hold",
    0, OnPlotModeChange2);

tCheckBoxWidget g_sPlotModeCheckBox = CheckBoxStruct(g_psPanels + 2,
    &g_sPlotModeCheckBox2, 0, &g_sKentec320x240x16_SSD2119,
    135, 200, 95, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Graph Switch",
    0, OnPlotModeChange);
    
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
Canvas(g_sCanvas3, g_psPanels + 3, &g_sPlotModeCheckBox3, 0,
       &g_sKentec320x240x16_SSD2119,
       30, 50, 200, 140,
       CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN,
       0, ClrGray, 0, 0, 0, 0, OnCanvasPaint);

tCheckBoxWidget g_sPlotModeCheckBox4 = CheckBoxStruct(g_psPanels + 3,
    NULL, 0, &g_sKentec320x240x16_SSD2119,
    30, 200, 65, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Hold",
    0, OnPlotModeChange2);

tCheckBoxWidget g_sPlotModeCheckBox3 = CheckBoxStruct(g_psPanels + 3,
    &g_sPlotModeCheckBox4, 0, &g_sKentec320x240x16_SSD2119,
    135, 200, 95, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Show Raw",
    0, OnPlotModeChange);

// Panel 4 - Plotting
Canvas(g_sCanvas4, g_psPanels + 4, &g_sPlotModeCheckBox6, 0,
       &g_sKentec320x240x16_SSD2119,
       30, 50, 200, 140,
       CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN,
       0, ClrGray, 0, 0, 0, 0, OnCanvasPaint);

tCheckBoxWidget g_sPlotModeCheckBox5 = CheckBoxStruct(g_psPanels + 4,
    NULL, 0, &g_sKentec320x240x16_SSD2119,
    30, 200, 65, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Hold",
    0, OnPlotModeChange2);

tCheckBoxWidget g_sPlotModeCheckBox6 = CheckBoxStruct(g_psPanels + 4,
    &g_sPlotModeCheckBox5, 0, &g_sKentec320x240x16_SSD2119,
    135, 200, 95, 20,
    CB_STYLE_OUTLINE | CB_STYLE_TEXT | CB_STYLE_SELECTED,
    16, 0, ClrGray, ClrGreen, &g_sFontCm16, "Show Raw",
    0, OnPlotModeChange);


// Canvas(g_sCanvas1, g_psPanels + 2, &g_sPlotModeCheckBox, 0,
//        &g_sKentec320x240x16_SSD2119, 230, 27, 90, 140,
//        CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_TEXT,
//        ClrMidnightBlue, ClrGray, ClrSilver, &g_sFontCm22, "Text", 0, 0);

// Main Panels Array
tCanvasWidget g_psPanels[5] =
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

    // Panel 3: Range / Plotting 2
    CanvasStruct(0, 0, &g_sCanvas3, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
                     // Panel 3: Range / Plotting 2
    CanvasStruct(0, 0, &g_sCanvas4, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 240, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
};

Canvas(g_sDashboard, g_psPanels, &g_sDashboardSlider, &g_sFaultPanel,
       &g_sKentec320x240x16_SSD2119, 0, 0,
       320, 240, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnDashboardPaint);

// // Big START Button (Red Circle)
// tPushButtonWidget g_sStartButton = CircularButtonStruct(&g_sDashboard, 0, 0,
//                                     &g_sKentec320x240x16_SSD2119, 90, 30, 55,    
//                                     PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
//                                     ClrRed, ClrBlack, ClrWhite, ClrWhite,
//                                     &g_sFontCm22, "START", 0, 0, 0, 0, OnStartButtonPress);

                                            // ====================== TIME CANVAS ON DASHBOARD ======================
tCanvasWidget g_sTimeCanvas = CanvasStruct( &g_sDashboard, 0, 0,
                                            &g_sKentec320x240x16_SSD2119,
                                            15, 8, 220, 28,                                      // x, y, w, h
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
                                            45, 160, 180, 200,
                                            CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN,
                                            ClrBlack, ClrRed, ClrWhite, &g_sFontCm16, "", 0, OnFaultPanelPaint);

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
                                    &g_sKentec320x240x16_SSD2119, 85, 65, 80, 55,     // square-ish 80x55
                                    PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
                                    ClrRed, ClrBlack, ClrWhite, ClrWhite,
                                    &g_sFontCm22, "START", 0, 0, 0, 0, OnStartButtonPress);


// ====================== PERMANENT SIDE BUTTONS ======================
// These are always on screen, on every panel

tPushButtonWidget g_psSideButtons[5] =
{
    // 0 - Engine
    RectangularButtonStruct(0, &g_psSideButtons[1], 0,
        &g_sKentec320x240x16_SSD2119, 235, 40, 75, 28,
        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
        ClrDarkBlue, ClrBlack, ClrWhite, ClrWhite,
        &g_sFontCm12, "Dash", g_pui8BtnUpImage, 0, 0, 0, OnDashButtonPress),

    // 1 - Light
    RectangularButtonStruct(0, &g_psSideButtons[2], 0,
        &g_sKentec320x240x16_SSD2119, 235, 75, 75, 28,
        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
        ClrDarkBlue, ClrBlack, ClrWhite, ClrWhite,
        &g_sFontCm12, "Engine", g_pui8BtnUpImage, 0, 0, 0, OnDashButtonPress),

    // 2 - Temp
    RectangularButtonStruct(0, &g_psSideButtons[3], 0,
        &g_sKentec320x240x16_SSD2119, 235, 110, 75, 28,
        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
        ClrDarkBlue, ClrBlack, ClrWhite, ClrWhite,
        &g_sFontCm12, "Light", g_pui8BtnUpImage, 0, 0, 0, OnDashButtonPress),

    // 3 - Range
    RectangularButtonStruct(0, 0, 0,
        &g_sKentec320x240x16_SSD2119, 235, 145, 75, 28,
        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
        ClrDarkBlue, ClrBlack, ClrWhite, ClrWhite,
        &g_sFontCm12, "Temp", g_pui8BtnUpImage, 0, 0, 0, OnDashButtonPress),
        // 3 - Range
    RectangularButtonStruct(0, 0, 0,
        &g_sKentec320x240x16_SSD2119, 235, 180, 75, 28,
        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY,
        ClrDarkBlue, ClrBlack, ClrWhite, ClrWhite,
        &g_sFontCm12, "Range", g_pui8BtnUpImage, 0, 0, 0, OnDashButtonPress),
};


// Plot callbacks
void OnPlotModeChange(tWidget *psWidget, uint32_t bSelected)
{
    g_ePlotMode = bSelected ? PLOT_MODE_RAW_AND_FILTERED : PLOT_MODE_FILTERED_ONLY;
    if (xEventGroup) xEventGroupSetBits(xEventGroup, EVENT_BTN_TOGGLE_PLOT);
    if (g_ui32Panel == 2) WidgetPaint((tWidget *)&g_sCanvas3);
}

void OnPlotModeChange2(tWidget *psWidget, uint32_t bSelected)
{
    g_bDisplayHold = bSelected;
    if (xEventGroup) xEventGroupSetBits(xEventGroup, EVENT_DISPLAY_HOLD);
    if (g_ui32Panel == 2) WidgetPaint((tWidget *)&g_sCanvas3);
}

void OnCanvasPaint(tWidget *psWidget, tContext *psContext)
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

    const char* modeStr = (g_ePlotMode == PLOT_MODE_RAW_AND_FILTERED) ?
                          "Raw + Filtered" : "Filtered Only";
    GrContextForegroundSet(psContext, ClrWhite);
    GrStringDrawCentered(psContext, modeStr, -1, canvas_xMin + canvas_xRange/2, 8, 0);

    // Draw plot lines
    for (ui32Idx = 0; ui32Idx < PLOT_HISTORY_SIZE - 1; ui32Idx++)
    {
        uint32_t dataIdx1 = (g_plotIndex + ui32Idx) % PLOT_HISTORY_SIZE;
        uint32_t dataIdx2 = (g_plotIndex + ui32Idx + 1) % PLOT_HISTORY_SIZE;

        uint32_t raw1  = g_plotRawHistory[dataIdx1];
        uint32_t raw2  = g_plotRawHistory[dataIdx2];
        uint32_t filt1 = g_plotFiltHistory[dataIdx1];
        uint32_t filt2 = g_plotFiltHistory[dataIdx2];

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
        
        if (g_ePlotMode == PLOT_MODE_RAW_AND_FILTERED)
        {   
            usprintf(buff2, "%4d",filt1); 
            CanvasTextSet(&g_sLightValCanvas2, buff2);
            WidgetPaint((tWidget *)&g_sLightValCanvas2);
            GrContextForegroundSet(psContext, ClrCyan);            
            GrLineDraw(psContext, x1, y1-3, x2, y2-3);
        }

        // GrContextFontSet(psContext, &g_sFontCm12);
        // GrContextForegroundSet(psContext, ClrYellow);
        // usprintf(buff,"%3d",(int32_t)filt1);
        // GrStringDrawCentered(psContext, buff, -1, 200, 65, 1);
        usprintf(buff1, "%4d",raw1); 
        CanvasTextSet(&g_sLightValCanvas, buff1);
        WidgetPaint((tWidget *)&g_sLightValCanvas);
        GrContextForegroundSet(psContext, ClrYellow);
        GrLineDraw(psContext, x1, y1f-3, x2, y2f-3);
    }
}

void OnNext(tWidget *psWidget)
{
    if (g_ui32Panel >= 2) return;

    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));
    g_ui32Panel++;
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));

    //CanvasTextSet(&g_sTitle, g_pcPanei32Names[g_ui32Panel]);
    //WidgetPaint((tWidget *)&g_sTitle);

    // Show/Hide Previous button
    if (g_ui32Panel == 1)
    {
        PushButtonImageOn(&g_sPrevious);
        PushButtonTextOn(&g_sPrevious);
        PushButtonFillOff(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }
}

void OnPrevious(tWidget *psWidget)
{
    if (g_ui32Panel == 0) return;

    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));
    g_ui32Panel--;
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));

    //CanvasTextSet(&g_sTitle, g_pcPanei32Names[g_ui32Panel]);
    //WidgetPaint((tWidget *)&g_sTitle);

    if (g_ui32Panel == 0)
    {
        PushButtonImageOff(&g_sPrevious);
        PushButtonTextOff(&g_sPrevious);
        PushButtonFillOn(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }
}

// Global value for the speed / gauge
// Temp hooks for later
//uint32_t g_ui32Speed = 80;        
uint32_t g_ui32Temp = 20;

void OnDashboardPaint(tWidget *psWidget, tContext *psContext)
{
    //tRectangle sRect;
    char speedtext[16];
    //char timetext[16];
    char temptext[16];

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
    

    GrContextFontSet(psContext, &g_sFontCm20);
    GrContextForegroundSet(psContext, ClrWhite);
    
    usprintf(temptext, "%d C", g_ui32Temp);
    GrStringDrawCentered(psContext, temptext, -1, 240, 20, 0);

    // Big Speed Number
    GrContextFontSet(psContext, &g_sFontCm34);
    GrContextForegroundSet(psContext, ClrCyan);
    usprintf(speedtext, "%d", g_ui32Speed);
    GrStringDrawCentered(psContext, speedtext, -1, 60, 75, 1);

    GrContextFontSet(psContext, &g_sFontCm16);
    GrStringDraw(psContext, "Km/H", -1, 85, 75, 0);

    UpdateMotorStateCanvas();
    UpdateFaultPanel();

    // Ensure slider is painted
    WidgetPaint((tWidget *)&g_sDashboardSlider);
    
}

void OnDashButtonPress(tWidget *psWidget)
{
    uint32_t targetPanel = g_ui32Panel;

    for (uint32_t i = 0; i < 5; i++)
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
    for (uint32_t i = 0; i < 5; i++)
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
        SetMotorState(MOTOR_STARTING);
    }
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

    GrStringDraw(psContext, "FAULTS & SAFETY", -1, 50, 165, 0);

    // Faults
    GrContextForegroundSet(psContext, (g_ui32ActiveFaults & FAULT_EBRAKE) ? ClrRed : ClrDarkGray);
    GrStringDraw(psContext, "EBRAKE", -1, 50, 180, 0);

    GrContextForegroundSet(psContext, (g_ui32ActiveFaults & FAULT_OVERCURRENT) ? ClrRed : ClrDarkGray);
    GrStringDraw(psContext, "OCURR", -1, 120, 180, 0);

    GrContextForegroundSet(psContext, (g_ui32ActiveFaults & FAULT_OVERTEMP) ? ClrRed : ClrDarkGray);
    GrStringDraw(psContext, "OTEMP", -1, 50, 200, 0);

    GrContextForegroundSet(psContext, (g_ui32ActiveFaults & FAULT_SENSOR) ? ClrRed : ClrDarkGray);
    GrStringDraw(psContext, "SENS", -1, 120, 200, 0);

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

void UpdateFaultPanel(void)
{
    WidgetPaint((tWidget *)&g_sFaultPanel);
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
        UpdateMotorStateCanvas();
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
    sRect.i16XMax = 320; sRect.i16YMax = 160;
    GrRectFill(psContext, &sRect);

    GrContextFontSet(psContext, &g_sFontCm16);
    GrContextForegroundSet(psContext, ClrWhite);

    // Live Values
    usprintf(buf, "Accel: %3lu", g_ui32FilteredBodyAccel);
    GrStringDraw(psContext, buf, -1, 70, 40, 0);

    usprintf(buf, "Distance: %3lu cm", g_ui32CurrentDistance);
    GrStringDraw(psContext, buf, -1, 70, 70, 0);

    // usprintf(buf, "Temp: %2lu C", g_ui32EngineTemp);
    // GrStringDraw(psContext, buf, -1, 70, 80, 0);

    usprintf(buf, "Crash Th: %4lu", g_ui32CrashThreshold);
    GrStringDraw(psContext, buf, -1, 70, 150, 0);

    // Paint the slider (important!)
    WidgetPaint((tWidget *)&g_sEngineSlider);
}

void OnEngineSliderChange(tWidget *psWidget, int32_t i32Value)
{
    static char pcSliderText[6];

    if (psWidget == (tWidget *)&g_sEngineSlider)
    {
        usprintf(pcSliderText, "%3d", (int)i32Value);
        SliderTextSet(&g_sEngineSlider, pcSliderText);

        // Update the variable you want to control
        g_ui32CrashThreshold = (uint32_t)i32Value * 10;   // Example scaling

        // ONLY repaint when slider actually changes
        if (g_ui32Panel == 1)
        {
            WidgetPaint((tWidget *)&g_sEngineCanvas);
        }
    }
}