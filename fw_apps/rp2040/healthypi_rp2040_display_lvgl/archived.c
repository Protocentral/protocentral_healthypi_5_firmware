//float y1_max = 0;
//float y1_min = 10000;

/*
void draw_plotppg(float data_ppg) {
  if (data_ppg < y1_min) {
    y1_min = data_ppg;

    plt1.setGraphScale(0.0, DISP_WINDOW_SIZE, y1_min, y1_max);
  }

  if (data_ppg > y1_max) {
    y1_max = data_ppg;
    plt1.setGraphScale(0.0, DISP_WINDOW_SIZE, y1_min, y1_max);
  }
  tr1.addPoint(gx, data_ppg);

  tr2.addPoint(gx, data_ppg);

  tr3.addPoint(gx, data_ppg);


  gx += 1;

  if (gx >= DISP_WINDOW_SIZE) {
    gx = 0.0;

    Serial.printf("y1 min: %d max: %d\n", y1_min, y1_max);



    plt1.drawGraph(0, DISP_PLOT1_Y_POS);
    tr1.startTrace(TFT_RED);

    plt2.setGraphScale(0.0, DISP_WINDOW_SIZE, y1_min, y1_max);
    plt2.drawGraph(0, DISP_PLOT2_Y_POS);
    tr2.startTrace(TFT_YELLOW);

    plt3.setGraphScale(0.0, DISP_WINDOW_SIZE, y1_min, y1_max);
    plt3.drawGraph(0, DISP_PLOT3_Y_POS);
    tr3.startTrace(TFT_GREEN);

    y1_min = 10000;
    y1_max = 0;
  }
}
*/

/*

void disp_value(uint8_t hr_val, uint8_t rr_val, uint8_t spo2_val, float temp_val) {

  tft.setTextSize(2);
  tft.setCursor(2, 303);
  tft.setTextColor(TFT_RED);
  tft.println("HR:");
  tft.setCursor(40, 303);
  tft.setTextColor(TFT_YELLOW);
  tft.println("65bpm");

  tft.setCursor(115, 303);
  tft.setTextColor(TFT_RED);
  tft.println("SPO2:");
  tft.setCursor(177, 303);
  tft.setTextColor(TFT_YELLOW);
  //tft.println(spo2_val);
  tft.println("96");

  tft.setCursor(212, 303);
  tft.setTextColor(TFT_RED);
  tft.println("Resp:");
  tft.setCursor(272, 303);
  tft.setTextColor(TFT_YELLOW);
  tft.println("20rpm");

  tft.setCursor(350, 303);
  tft.setTextColor(TFT_RED);
  tft.println("Temp:");
  tft.setCursor(410, 303);
  tft.setTextColor(TFT_YELLOW);
  //tft.println(temp_val);
  tft.println("35.2");
}
*/

/*void draw_plotecg(float data_ecg)
{
   static float gx = 0.0;

   float availableSpace = 220.0; // the size of your canvas
   float dataRange = 60;      // the range of your values
   float scaleFactor = availableSpace / dataRange;

   //float plottableEY = availableSpace - (data_ecg * scaleFactor);
   tr1.addPoint(gx, data_ecg); 
   gx += 0.1;

   if (gx > 100.0) {
      gx = 0.0;
    
    gr.drawGraph(5, 65);
    tr.startTrace(TFT_GREEN);
    }
    
}*/
/*
void tft_plot_setup()
{
  /*gr.createGraph(460, 65, tft.color565(5, 5, 5));
  gr.setGraphScale(0.0, 100.0, -80.0, 80.0);
 // gr.setGraphGrid(0.0, 10.0, -40.0, 50, TFT_SILVER);   
  gr.setGraphPosition(5, 65);
  gr.drawGraph(5, 65);

  tr.addPoint(0.0,0.0);
  tr.addPoint(100.0, 0.0);

  tr.startTrace(TFT_WHITE);
  
  plt1.createGraph(460, DISP_GRAPH_HGT, tft.color565(0, 0, 0));
  plt1.setGraphScale(0.0, DISP_WINDOW_SIZE, 0.0, DISP_GRAPH_HGT);
  plt1.drawGraph(0, DISP_PLOT1_Y_POS);

  //y1_min = 10000;
  //y1_max = 0;

  plt2.createGraph(460, DISP_GRAPH_HGT, tft.color565(0, 0, 0));
  plt2.setGraphScale(0.0, DISP_WINDOW_SIZE, 0.0, DISP_GRAPH_HGT);
  plt2.drawGraph(0, DISP_PLOT2_Y_POS);

  plt3.createGraph(460, DISP_GRAPH_HGT, tft.color565(0, 0, 0));
  plt3.setGraphScale(0.0, DISP_WINDOW_SIZE, 0.0, DISP_GRAPH_HGT);
  plt3.drawGraph(0, DISP_PLOT3_Y_POS);

  tr1.startTrace(TFT_RED);
  //tr1.addPoint(0.0, 0.0);
  //tr1.addPoint(100.0, 0.0);

  tr2.startTrace(TFT_YELLOW);
  tr2.addPoint(0.0, 0.0);
  tr2.addPoint(100.0, 0.0);

  tr3.startTrace(TFT_GREEN);
  tr3.addPoint(0.0, 0.0);
  tr3.addPoint(100.0, 0.0);
}
*/
/*
GraphWidget plt1 = GraphWidget(&tft);
TraceWidget tr1 = TraceWidget(&plt1);
GraphWidget plt2 = GraphWidget(&tft);
TraceWidget tr2 = TraceWidget(&plt2);
GraphWidget plt3 = GraphWidget(&tft);
TraceWidget tr3 = TraceWidget(&plt3);

#define DISP_WIDTH 480
#define DISP_HGT 320
#define SAMPLE_RATE 125

#define DISP_WINDOW_SIZE SAMPLE_RATE * 6

#define DISP_HEADER_HGT 40
#define DISP_FOOTER_HGT 20

#define DISP_GRAPH_HGT (DISP_HGT - DISP_FOOTER_HGT - DISP_HEADER_HGT) / 3

#define DISP_PLOT1_Y_POS DISP_HEADER_HGT + 1
#define DISP_PLOT2_Y_POS DISP_HEADER_HGT + DISP_GRAPH_HGT + 2
#define DISP_PLOT3_Y_POS DISP_HEADER_HGT + (DISP_GRAPH_HGT * 2) + 3
*/