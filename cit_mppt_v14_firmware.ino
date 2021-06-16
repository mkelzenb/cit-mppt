
#include <ADC.h>
#include <ADC_util.h>
#include <math.h>
#include <EEPROM.h>
#include <string.h>
#include <TimeLib.h>


#define num_chans 12
#define default_res1 101.0L
#define default_res2 10.0L
#define flag_ee_auto_run 0x01
#define isAutoRun (ee.auto_flags & flag_ee_auto_run)
//#define flag_ee_auto_rapid 0x02
#define flag_ee_auto_save 0x04
#define isAutoSave (ee.auto_flags & flag_ee_auto_save)
#define flag_ee_echo 0x8
#define isEcho (ee.auto_flags & flag_ee_echo)
#define flag_ee_RTC 0x10
#define useRTC (ee.auto_flags & flag_ee_RTC)
#define flag_ee_print_header 0x20
#define shouldPrintHeader (ee.auto_flags & flag_ee_RTC)
#define adc_bits 16
#define dac_bits 12
const uint16_t dac_max = (1<<dac_bits)-1;
#define dac_range 3.30L
#define dac_min_sw_step 4
#define v_min_sw_step 0.00322265625L
#define eeprom_test_addr 4095
#define default_res 100.1L
#define ser_buff_length 128 
#define mppt_hist_depth 8
#define max_cmd_pars 10
#define max_sw_pts 254
#define max_sw_cycles 5
#define max_sw_delays 1000
#define max_avgs 1024


#define pm_dis  0
#define pm_mppt 1
#define pm_voc  2
#define pm_isc  3
#define pm_hold 4
#define pm_cv   5
#define pm_cc   6
#define pm_ext  7
#define pm_idle 8
const char *modeStrs[9] = { "DISABLE", "MPPT", "VOC", "ISC", "HOLD", "CV", "CC", "EXTERNAL", "IDLE" };

#define sm_stop 0
#define sm_op   1
#define sm_hold 2

char ser_buff[ser_buff_length+1];
char cmdstr[ser_buff_length+1];
int ser_pos = 0;
char incomingByte;
char inputValid;

struct eeDataStruct
{
  uint16_t num_avgs;  // default 1.
  uint16_t file_num;   // this is the NEXT safe file number.  Always increment on successful startup.
  uint16_t loop_delay_ms;
  uint16_t bias_delay_ms;
  uint16_t ch_delay_ms;
  uint16_t relay_delay_ms;
  uint16_t auto_flags; // see flags above
  uint8_t min_mppt_step_bits; // min dac step (x DAC LSB) for MPPT
  uint8_t fast_mppt_step_bits; // max dac step (these bits plus min_mppt_step_bits bits is maximum DAC step size for MPPT)
  uint8_t max_vsearch_step_bits; // max dac bit step size for initial voltage search
  uint16_t unused; // for the FUTURE
  uint64_t ch_enable;
  uint64_t px_ressel;
  float px_res1[num_chans];
  float px_res2[num_chans];
  float px_cal1[num_chans];
  float px_cal2[num_chans];
  byte px_mode[num_chans];
} ee;

//system variables
uint64_t numiters = 0;  // number of primary read loops
uint64_t line_num = 0;  // number of output cycles (to terminal)
uint64_t skip_output_interval = 0;  // how many read loops to skip between outputting to terminal
int polarity = 1; // this means common hi.  OPPOSITE POLARITY IS NOT YET SUPPORTED!
byte mode; // system operating mode

// pin maps
const uint8_t pins_adc1[] = {  A0,  A1,  A2,  A3,  A4,  A5,  A6,  A7,  A8,  A9, A10, A21 };   //DID THIS:   should have swapped A11 with A21!
const uint8_t pins_adc2[] = { A12, A13, A14, A15, A17, A18, A19, A20, A11, A22, A23, A24 };  //             then could have done synchronous.
const uint8_t pins_dac[] =  {   2,   3,   4,   5,   6,   7,   8,   9,  10,  29,  30,  35 };  //             so instead I can't do sync on 8 and 11.
const uint8_t pins_reed[] = {  45,  46,  47,  48,  51,  52,  53,  54,  55,  56,  57,  13 };
const uint8_t pins_ext[]  = {  11,  12,  24,  25,  26,  27,  28,  40,  41,  42,  43,  44 };
const uint8_t pins_sync[] = {   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1 };

// global data passing for readADCs()
double adcvolts1;
double adcvolts2;
byte adccomp;

// pixel data / state buffers
double volts1[num_chans];
double volts2[num_chans];
double volts[num_chans];  // this is the corrected value (todo:  including cal offsets)
double res[num_chans];  // this is a local copy of the current resistances
double curr[num_chans];  // calculated currents
double bias[num_chans];
double pwr[num_chans];
double voltsL[num_chans];
double currL[num_chans];
double voltsH[num_chans];
double currH[num_chans];
double biasL[num_chans];
double biasH[num_chans];
double pwrH[num_chans];
double pwrL[num_chans];
double cal_diff[num_chans];
double cal_V1[num_chans];
double vtarg[num_chans];  // target voltage for CV mode
double itarg[num_chans];  // target current for CC mode
byte changedTarget[num_chans];   // set to 1 if the CV/CC targets have abruptly changed, for faster initial slewing
byte px_mode[num_chans];  // pixel mode
byte px_prior_mode[num_chans];  // pixel mode
short mppt_hist[num_chans][mppt_hist_depth];  // search direction history buffer for MPPT, for increasing step size following abrupt change in light intensity
byte px_reed[num_chans]; //convenience variable, is reed relay closed?
byte px_ext[num_chans]; //convenience variable, is external instrument relay energized?
byte px_compl[num_chans];   // is channel overloaded / ADC out of range ("compliance")?

//for command processing
byte px_sel[num_chans];
double cmd_pars[max_cmd_pars];
uint8_t num_cmd_pars;

double sw_volts[max_sw_pts];
double sw_curr[max_sw_pts];
double sw_bias[max_sw_pts];
double sw_pwr[max_sw_pts];
double sw_times[max_sw_pts];
uint8_t sw_pts;

// ADC/DAC constants
const double adc_vstep = (dac_range / (1<<adc_bits) );
const double dac_vstep = dac_range / (1<<dac_bits );
const uint16_t dac_n_3V = ( (3.0L / dac_range) * (1<<dac_bits) );
const uint16_t dac_n_p5V = ( (0.5L / dac_range) * (1<<dac_bits) );
const double v_com = 3.0L;  //dac_n_3V * adc_vstep;
const double max_bias = v_com;
const double min_bias = (v_com - dac_range);

// timekeeping variables
unsigned long tstart;
unsigned long tic;
unsigned long toc;
unsigned long tmppt;

byte printedHeader=0;

ADC *adc = new ADC(); // adc object

void setupEEPROMDefaults();

void setup() {

  // Set up Serial
  Serial.begin(9600);  // teensy serial is always 12 MBPS

  //this forces the program to wait until a serial connection is established
  while(!Serial)
  { delay(5); }
  Serial.println("MK MPPTx12 V44 DEVELOPMENT FIRMWARE, MARCH 5 2021");

  // initialize variables with default values 
  for (int m=0; m<num_chans; m++)
  {
     // if (ee.px_ressel[m])
     //   res[m] = ee.px_res2[m];
     // else
     //   res[m] = ee.px_res1[m];
      res[m] = 101.0;
      px_mode[m]=pm_dis;
      for (int n=0; n<mppt_hist_depth; n++)
        mppt_hist[m][n]=0;

      cal_V1[m]=0;
      cal_diff[m]=0;
      vtarg[m] = 0;
      itarg[m] = 0;
      changedTarget[m] = 0;
  }

  // Set up EEPROM
    byte eeprom_testval;
//    EEPROM.get(eeprom_test_addr, eeprom_testval );
//    if ( eeprom_testval != num_chans ){
      setupEEPROMDefaults();
//      EEPROM.put(0,ee);
//      eeprom_testval = num_chans;
//      EEPROM.put(eeprom_test_addr, eeprom_testval );
      Serial.println("Unrecognized EEPROM state.  Reverting to default values.");
//    }
/*    else  {
      EEPROM.get(0,ee);
      Serial.print("Read settings from EEPROM (");
      Serial.print(sizeof(ee),DEC);
      Serial.println(" bytes)");
    }
*/

  // apply EEPROM values to local settings   
  for (int m=0; m<num_chans; m++)
  {
     // if (ee.px_ressel[m])
     //   res[m] = ee.px_res2[m];
     // else
     //   res[m] = ee.px_res1[m];
  }

  // Set up ADCs
  adc->adc0->setReference(ADC_REFERENCE::REF_EXT);
  adc->adc0->setResolution(adc_bits);
  adc->adc0->setAveraging(ee.num_avgs);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED );

  adc->adc1->setReference(ADC_REFERENCE::REF_EXT);
  adc->adc1->setResolution(adc_bits);
  adc->adc1->setAveraging(ee.num_avgs);
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED );

  // set up DACs  
  analogWriteResolution(12);
  for (int nni = 0; nni < num_chans; nni++)
  {
    adc->resetError() ;
    analogWriteFrequency(pins_dac[nni],  14648.437 );
    pinMode(pins_reed[nni], OUTPUT);
    pinMode(pins_ext[nni], OUTPUT );

    digitalWrite(pins_reed[nni],0);
    digitalWrite(pins_ext[nni],0);

    //start with short circuit condition
    analogWrite(pins_dac[nni], v_com/dac_vstep );
    bias[nni] = 0;
    vtarg[nni] = 0;
    // also... applyBias(nni,0);
    /*Serial.print("Set CH");
    Serial.print(nni);
    Serial.print(" DAC to ");
    Serial.print((uint16_t)(v_com/dac_vstep),BIN);
    Serial.print(" / ");
    Serial.println((uint16_t)(v_com/dac_vstep));*/
    
    if(adc->adc0->fail_flag != ADC_ERROR::CLEAR) {
      Serial.print("ADC0: "); Serial.println(getStringADCError(adc->adc0->fail_flag));
    }
    #ifdef ADC_DUAL_ADCS
    if(adc->adc1->fail_flag != ADC_ERROR::CLEAR) {
      Serial.print("ADC1: "); Serial.println(getStringADCError(adc->adc1->fail_flag));
    }
    #endif
  }

  //temporary setup, put us in MPPT mode
  for (int m = 0; m<num_chans; m++)
  {
    px_mode[m] = pm_mppt;
    px_reed[m] = 1;
    digitalWrite(pins_reed[m],1);
  }
  
  delay(100); // relay time

  //   self_cal();
  //hard-coded cal values for now, due to design deficiency in V0 boards
  /*cal_V1[0] = 1.24e-3;
  cal_V1[1] = 0.24e-3;
  cal_V1[2] = 1.87e-3;
  cal_V1[3] = 2.67e-3;
  cal_diff[0] = 3e-5;
  cal_diff[1] = 4e-5;
  cal_diff[2] = 1e-5;
  cal_diff[3] = -0e-5;
*/
  //self_cal();  //doesn't work :(

  mode=1;
  inputValid=1;

  delay(100);

  Serial.println("Starting operation...");
  Serial.println("");
  if (skip_output_interval) printHeader();
  
  tstart = millis();


  uint32_t adc1_val = 0;
  uint32_t adc2_val = 0;
  ADC::Sync_result asr;
  /*
       asr = adc->analogSyncRead(pins_adc1[5], pins_adc2[5]);
       adc1_val += (uint16_t)asr.result_adc0;
       adc2_val += (uint16_t)asr.result_adc1;
       Serial.print("Sync result:\t");
       Serial.print(asr.result_adc0);
       Serial.print("\t");
       Serial.println(asr.result_adc1);

     Serial.println(getStringADCError((ADC_Error::ADC_ERROR)asr.result_adc0));
     Serial.println(asr.result_adc0,BIN);
     Serial.println(adc1_val, BIN);
  
    adc1_val = adc->analogRead(pins_adc1[5]);
    adc2_val = adc->analogRead(pins_adc2[5]);
       Serial.print("Norm result:\t");
       Serial.print(adc1_val);
       Serial.print("\t");
       Serial.println(adc2_val);

       Serial.println(adc1_val,BIN);
    */   
       

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// printHeader()
///////////////////////////////////////////////////////////////////////////////////////////////////
void printHeader()
{
  if (shouldPrintHeader)
  {
  Serial.print("Line\tt \t\t");
  for (int n=0; n<num_chans; n++)
  {
    if (px_mode[n])
    {
      Serial.print("CH ");
      Serial.print(n);
      Serial.print("\t");
      switch (px_mode[n])
      {
        case pm_mppt:
          Serial.print("MPPT");
          break;
        case pm_voc:
          Serial.print("Voc");
          break;
        case pm_isc:
          Serial.print("Isc");
          break;
        case pm_cv:
          Serial.print("CV");
          break;
        case pm_cc:
          Serial.print("CC");
          break;
        case pm_hold:
          Serial.print("Hold");
          break;
        case pm_ext:
          Serial.print("External");
          break;
        case pm_idle:
          Serial.print("Idle");  
      }
      Serial.print("\t\t");
    }
  }
  Serial.println("");
  Serial.print("num\t(s)\t\t");
  for (int n=0; n<num_chans; n++)
  {
    if (px_mode[n])
    {
      Serial.print("V\t");
      Serial.print("mA\t\t");
    }
  }
  Serial.println("");

  /*//test
  uint16_t gotBiasDAC = (v_com-bias[5]) / dac_vstep;
  Serial.print("Testing... getBiasDAC(5) = ");
  Serial.print(gotBiasDAC,BIN);
  Serial.print("  /  ");
  Serial.print(gotBiasDAC);
  Serial.print("\twhich is ");
  Serial.print(gotBiasDAC * dac_vstep, 5);
  Serial.print("\t(");
  Serial.print(v_com - gotBiasDAC*dac_vstep, 5);
  Serial.println(")");
  Serial.print("Also, dac_max=");
  Serial.println(dac_max);

  char *s = "-1.61e4,123.45,0x78c3";

  double d = atof(s);

  Serial.println(d,8);*/

/*
  char tmpbuff[1024];
  sprintf(tmpbuff, "Testing teensy sprintf():  q0 = %g, pi = %g, and NA = %g\n", 1.61e-19, 3.14156, 6.02214076e23);
  Serial.println(tmpbuff);
  Serial.printf( "Testing Serial.printf():  q0 = %g, pi = %g, and NA = %g\n", 1.61e-19, 3.14156, 6.02214076e23);
*/  
  }
}







void loop() 
{
  
  //////////////////////////////////////////////////////////////////////////////////////
  //  read serial commands
  //////////////////////////////////////////////////////////////////////////////////////
  while (SerialUSB.available() > 0) 
  {
    incomingByte = SerialUSB.read();
    if (incomingByte == '\n')
    {
     // process command!
     if (inputValid)
     {
/**///                                   Serial.print("Command received: \"");
/**///                                   for (int m=0; m<ser_pos; m++)
/**///                                   {
/**///                                      Serial.print(ser_buff[m]);
/**///                                   }
/**///                                   Serial.println("\"");
       int psresult = processPxSpec();
/**///                                   Serial.print("PX Spec processor return code: ");
/**///                                   Serial.println(psresult);
/**///                                   for (int i=0; i<num_chans; i++)
/**///                                   { Serial.print(i); Serial.print("\t"); }
/**///                                   Serial.println("");
/**///                                   for (int i=0; i<num_chans; i++)
/**///                                   { Serial.print(px_sel[i]); Serial.print("\t"); }
/**///                                   Serial.println("");
                                   
/**///                                   Serial.print("Remaining command string: ");
/**///                                   int k=psresult;
/**///                                   while( ser_buff[k] > 0)
/**///                                   {
/**///                                     Serial.print(ser_buff[k++]);
/**///                                   }
/**///                                   Serial.println("");
/**///                                   Serial.println("");
/**///                                   Serial.println("");
         if (psresult < 0)
         {  
            Serial.print("Command error:  Malformed pixel address.  (Code ");
            Serial.print(psresult);
            Serial.println(")");
            ser_buff[0] = 0;
            ser_pos=0;
            break;
         } 


         ////  get command string ////////////////////////////////////////////////
         
         char* remainingCmd = &ser_buff[psresult];
/**///                                                                      Serial.print("---> remaining string ");
/**///                                                                      Serial.println(remainingCmd);

         int i;
         for (i=0; i<ser_buff_length; i++)
         if ( (remainingCmd[i] != '?') && ((remainingCmd[i] < 'A') || (remainingCmd[i] > 'Z' ) ) )
            break;

         //char cmdstr[i+1];
         cmdstr[i] = 0;
         int j;
         for (j=0; j<i; j++) cmdstr[j] = remainingCmd[j];
         cmdstr[j] = 0;
/**///                                                                      Serial.print("Command: \""); 
/**///                                                                      Serial.print(cmdstr);  Serial.println("\"");

         while ( isWhitespace(remainingCmd[i]) || (remainingCmd[i]==',') || (remainingCmd[i]==':') ) 
          i++;

         remainingCmd = &remainingCmd[i];

/**///                                                                     Serial.print("---> remaining string: ");
/**///                                                                     Serial.println(remainingCmd);
 
         ////  get parameters  ////////////////////////////////////////////////

         char par_buff[strlen(remainingCmd)];
         
         for (num_cmd_pars = 0; num_cmd_pars < max_cmd_pars; num_cmd_pars++)
         {
            i=0;
            while ( (remainingCmd[i]) && (!isWhitespace(remainingCmd[i])) && (remainingCmd[i] != ',') && (remainingCmd[i] != ':')  )
            { 
              par_buff[i] = remainingCmd[i]; 
              i++;
            }
            par_buff[i] = 0;
/**///                                                                      Serial.print("Par string "); Serial.print(num_cmd_pars+1); Serial.print(" ("); 
/**///                                                                      Serial.print(i); Serial.print("): "); Serial.println(par_buff);
            if (i > 0)
            {
              cmd_pars[num_cmd_pars] = atof(par_buff);                
/**///                                                                      Serial.printf("Par val %d: %g\n", num_cmd_pars, cmd_pars[num_cmd_pars]);
              while ( isWhitespace(remainingCmd[i]) || (remainingCmd[i]==',') || (remainingCmd[i]==':') ) 
                i++;
              remainingCmd = &remainingCmd[i];                        
/**///                                                                      Serial.print("---> remaining string: ");   Serial.println(remainingCmd);
            }
            else
            {                                                         
/**///                                                                      Serial.printf("breaking at numCmdPars=%d\n",num_cmd_pars); 
              break;
            }
         }
/**///                                                                      Serial.printf("I found %d parameters.  Oh, and also, q0 = %g C\n",num_cmd_pars,1.61e-19);
         if (remainingCmd <= &ser_buff[ser_pos])
         {
            Serial.printf("WARNING:  To many parameters were given.   %d bytes of the command string were ignored.\n", (&ser_buff[ser_pos] - remainingCmd) );
         }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //  Command execution 
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        int num_used_pars = 0;
        byte invalid_cmd = 0;
        byte px_prior_reed[num_chans];
        byte px_prior_ext[num_chans];
        double px_prior_bias[num_chans];
        byte modechanged = 0;
        
        for (int n=0; n<num_chans; n++)
        {
            px_prior_mode[n] = px_mode[n];
            px_prior_reed[n] = digitalRead(pins_reed[n]);
            px_prior_ext[n] = digitalRead(pins_ext[n]);
            px_prior_bias[n] = bias[n];
        }
        
        if (psresult>1)  // this means we have pixel-specific commands
        {

          for (int n=0; n<num_chans; n++)
          {
//            px_prior_mode[n] = px_mode[n];
//            px_prior_reed[n] = digitalRead(pins_reed[n]);
//            px_prior_ext[n] = digitalRead(pins_ext[n]);
//            px_prior_bias[n] = bias[n];
            
            
            if (px_sel[n])
            {
/**///                                                                      Serial.printf("P%d",n);               

              if ( strcmp(cmdstr, "?") == 0 )          ////////////////////  ? (print)    ///////////////////////////////////////////////////
              {
/**///                                                                          Serial.println(".?");
                printPx(n);
                num_used_pars = 0;
              }
              else if ( strcmp(cmdstr, "IDLE") == 0 )          ////////////////////  IDLE    ///////////////////////////////////////////////////
              {
/**///                                                                      Serial.println(".IDLE");                 
                px_mode[n] = pm_idle;  
                px_ext[n] = 0;
                px_reed[n] = 0;
                digitalWrite(pins_ext[n],0);
                digitalWrite(pins_reed[n],0);
                num_used_pars = 0;
              }
              else if ( strcmp(cmdstr, "HOLD") == 0 )          ////////////////////  HOLD    ///////////////////////////////////////////////////
              {
/**///                                                                      Serial.println(".HOLD");                 
                num_used_pars = 0;
                if (num_cmd_pars>0) // provided bias value
                {
                  if ( (cmd_pars[0] <= max_bias) && (cmd_pars[0] >= min_bias) )
                  {
/**///                                                                      Serial.printf("  with voltage %g V \n",cmd_pars[0]);                      
                    num_used_pars = 1;
                    bias[n] = cmd_pars[0];
                    analogWrite(pins_dac[n],(v_com-cmd_pars[0])/dac_vstep);
                  }
                  else
                  {
                    Serial.printf("ERROR: Invalid bias value for HOLD command (%g V)\n",cmd_pars[0]);
                    invalid_cmd = 1;
                   break;
                  }
                }             
                px_mode[n] = pm_hold;  
                px_ext[n] = 0;
                px_reed[n] = 1;
                digitalWrite(pins_ext[n],0);
                digitalWrite(pins_reed[n],1);
              }
              else if ( ( strcmp(cmdstr, "MPPT") == 0 )  )  ////////////////////  MPPT    ///////////////////////////////////////////////////
              {
/**///                                                                      Serial.println(".MPPT");    
                num_used_pars = 0;
                if (num_cmd_pars>0) // provided value
                {
                  if ( (cmd_pars[0] <= max_bias) && (cmd_pars[0] >= min_bias) )
                  {
/**///                                                                      Serial.printf("  with voltage %g V \n",cmd_pars[0]);                      
                    num_used_pars = 1;
                    bias[n] = cmd_pars[0];
                    analogWrite(pins_dac[n],(v_com-cmd_pars[0])/dac_vstep);
                    
                  }
                  else
                  {
                    Serial.printf("ERROR: Invalid bias value for MPPT command (%g V)\n",cmd_pars[0]);
                    invalid_cmd = 1;
                    break;
                  }
                }             
                px_mode[n] = pm_mppt;  
                px_ext[n] = 0;
                px_reed[n] = 1;
                digitalWrite(pins_ext[n],0);
                digitalWrite(pins_reed[n],1);
              }
              else if ( ( strcmp(cmdstr, "DIS") == 0 ) || ( strcmp(cmdstr, "DISABLE") == 0 ) )  ////////////////////  DISABLE    ///////////////
              {
/**///                                                                      Serial.println(".DISABLE");                 
                px_mode[n] = pm_dis;  
                px_ext[n] = 0;
                px_reed[n] = 0;
                digitalWrite(pins_ext[n],0);
                digitalWrite(pins_reed[n],0);
                num_used_pars = 0;
              }
              else if ( ( strcmp(cmdstr, "VOC") == 0 )  )  ////////////////////  VOC    ///////////////////////////////////////////////////
              {
/**///                                                                      Serial.println(".VOC");                 
                px_mode[n] = pm_voc;  
                px_ext[n] = 0;
                px_reed[n] = 0;
                digitalWrite(pins_ext[n],0);
                digitalWrite(pins_reed[n],0);
                itarg[n] = 0;
                num_used_pars = 0;
              }
              else if ( ( strcmp(cmdstr, "ISC") == 0 ) )    ////////////////////  ISC    ///////////////////////////////////////////////////
              {
/**///                                                                      Serial.println(".ISC");                 
                num_used_pars = 0;
                if (num_cmd_pars>0) // provided guess for bias
                {
                  if ( (cmd_pars[0] <= max_bias) && (cmd_pars[0] >= min_bias) )
                  {
/**///                                                                      Serial.printf("  with bias guess %g (DAC=%d)\n",cmd_pars[0],(v_com-cmd_pars[0])/dac_vstep);                      
                    num_used_pars = 1;
                    analogWrite(pins_dac[n], (v_com-cmd_pars[0])/dac_vstep );
                    bias[n] = cmd_pars[0];
                  }
                  else
                  {
                    Serial.printf("ERROR: Invalid bias guess for ISC command (%g V)\n",cmd_pars[0]);
                    invalid_cmd = 1;
                    break;
                  }
                }
                else
                {
                  analogWrite(pins_dac[n], v_com/dac_vstep );
                  bias[n] = 0;                 
                }
                px_mode[n] = pm_isc;  
                px_ext[n] = 0;
                px_reed[n] = 1;
                digitalWrite(pins_ext[n],0);
                digitalWrite(pins_reed[n],1);
                vtarg[n] = 0;
              }
              else if ( ( strcmp(cmdstr, "CV") == 0 ) )   ////////////////////  CV   ///////////////////////////////////////////////////
              {
/**///                                                                      Serial.println(".CV");                 
                num_used_pars = 0;
                double oldvtarg = vtarg[n];
                if (num_cmd_pars>0) // provided value
                {
                  if ( (cmd_pars[0] <= max_bias) && (cmd_pars[0] >= min_bias) )
                  {
/**///                                                                      Serial.printf("  with voltage %g V \n",cmd_pars[0]);                      
                    num_used_pars = 1;
                    vtarg[n] = cmd_pars[0];
                  }
                  else
                  {
                    Serial.printf("ERROR: Invalid value for VC command (%g)\n",cmd_pars[0]);
                    invalid_cmd = 1;
                    break;
                  }
                }
                if (num_cmd_pars>1) // provided bias guess
                {
                  if ( (cmd_pars[1] <= max_bias) && (cmd_pars[1] >= min_bias) )
                  {
/**///                                                                      Serial.printf("  with bias %g (DAC=%d)\n",cmd_pars[0],(v_com-cmd_pars[0])/dac_vstep);                      
                    num_used_pars = 2;
                    analogWrite(pins_dac[n], (v_com-cmd_pars[1])/dac_vstep );
                    bias[n] = cmd_pars[1];
                  }
                  else
                  {
                    Serial.printf("ERROR: Invalid bias guess for VC command (%g)\n",cmd_pars[1]);
                    vtarg[n] = oldvtarg;
                    invalid_cmd = 1;
                    break;
                  }
                }
                else
                {
                  //ok, nothing else to do, switching to cv mode with prior vtarg value...                 
                }
                px_mode[n] = pm_cv;  
                px_ext[n] = 0;
                px_reed[n] = 1;
                digitalWrite(pins_ext[n],0);
                digitalWrite(pins_reed[n],1);
              }
              else if ( ( strcmp(cmdstr, "CC") == 0 ) )   ////////////////////  CC   ///////////////////////////////////////////////////
              {
/**///                                                                      Serial.println(".CC");                 
                num_used_pars = 0;
                double olditarg = itarg[n];
                if (num_cmd_pars>0) // provided value
                {
                  double max_curr = (max_bias - min_bias)/res[n];
                  if ( (cmd_pars[0]/1000 <= max_curr) && (cmd_pars[0]/1000 >= -max_curr) )
                  {
/**///                                                                      Serial.printf("  with current %g mA \n", cmd_pars[0]);                      
                    num_used_pars = 1;
                    itarg[n] = cmd_pars[0]/1000;
                  }
                  else
                  {
                    Serial.printf("ERROR: Invalid value for CC command (%g mA)\n",cmd_pars[0]);
                    invalid_cmd = 1;
                    break;
                  }
                }
                if (num_cmd_pars>1) // provided bias guess
                {
                  if ( (cmd_pars[1] <= max_bias) && (cmd_pars[1] >= min_bias) )
                  {
/**///                                                                      Serial.printf("  with bias guess %gV (DAC=%d)\n",cmd_pars[0],(v_com-cmd_pars[0])/dac_vstep);                      
                    num_used_pars = 2;
                    analogWrite(pins_dac[n], (v_com-cmd_pars[1])/dac_vstep );
                    bias[n] = cmd_pars[1];
                  }
                  else
                  {
                    Serial.printf("ERROR: Invalid bias guess for CC command (%g).  Max = %g V, min = %g V.\n",cmd_pars[1], max_bias, min_bias);
                    itarg[n] = olditarg;
                    invalid_cmd = 1;
                    break;
                  }
                }
                px_mode[n] = pm_cc;  
                px_ext[n] = 0;
                px_reed[n] = 1;
                digitalWrite(pins_ext[n],0);
                digitalWrite(pins_reed[n],1);
              }
              else if ( ( strcmp(cmdstr, "EXT") == 0 ) )  ////////////////////  EXT    ///////////////////////////////////////////////////
              {
/**///                                                                      Serial.println(".EXT");               
                px_mode[n] = pm_ext;  
                px_ext[n] = 1;
                //px_reed[n] = 1;
                digitalWrite(pins_ext[n],1);
                //digitalWrite(pins_reed[n],1);
                num_used_pars = 0;
                //analogWrite(pins_dac[n], v_com/dac_vstep );
                //bias[n] = 0;
                //vtarg[n] = 0;
              }
              ///////////////////////////////////////////////////////////////// SWEEP ///////////////////////////////////////////////////////
              else if ( ( strcmp(cmdstr, "VSW") == 0 ) || ( strcmp(cmdstr, "VSWEEP") == 0 ) || ( strcmp(cmdstr, "SWEEP") == 0 ) || ( strcmp(cmdstr, "SW") == 0 ) ||
                        ( strcmp(cmdstr,  "BSW") == 0 ) || ( strcmp(cmdstr, "BSWEEP") == 0 )            )              
              {
              
                int isbsw = ( ( strcmp(cmdstr, "BSW") == 0 ) || ( strcmp(cmdstr, "BSWEEP") == 0 ) );
                double sw_start = min_bias;
                double sw_end = max_bias;
                int sw_npts = 21;
                double sw_step = (sw_end - sw_start) / (sw_npts-1);
                uint8_t sw_numcycles = 0;
                uint8_t sw_extradelays = 0;
                byte sw_reedrelaychanged = 0;
                byte sw_extrelaychanged = 0;

                //check params and set up sweep
                if (num_cmd_pars == 1)
                {
                  Serial.printf("ERROR: Invalid number of parameters for sweep command.\n");
                  invalid_cmd = 1;
                  break;
                }
                if (num_cmd_pars > 1) // sweep start and stop
                {
                  if ( (cmd_pars[1] > max_bias) || (cmd_pars[1] < min_bias) || (cmd_pars[0] > max_bias) || (cmd_pars[0] < min_bias) ) 
                  {
                    Serial.printf("ERROR: Invalid value for sweep start/stop (%g, %g)\n",cmd_pars[0], cmd_pars[1]);
                    invalid_cmd = 1;
                    break;
                  }
                  else
                  {
                    num_used_pars = 2;
                    sw_start = cmd_pars[0];
                    sw_end = cmd_pars[1];
                  }
                }
                if (num_cmd_pars > 2)  // sweep number of points
                {
                  sw_npts = cmd_pars[2];
                  if ( ( sw_npts < 2) || (sw_npts > max_sw_pts ) )
                  {
                    Serial.printf("ERROR: Invalid number of sweep points (%d)\n",sw_npts);
                    invalid_cmd = 1;
                    break;
                  }
                  else
                  {
                    num_used_pars = 3;
                  }
                }
                if (num_cmd_pars > 3)  // sweep number of cycles
                {
                  sw_numcycles = cmd_pars[3];
                  if ( (sw_numcycles < 0) || (sw_numcycles > max_sw_cycles ) )
                  {
                    Serial.printf("ERROR: Invalid number of sweep cycles (%d)\n",sw_numcycles);
                    invalid_cmd = 1;
                    break;
                  }
                  else
                  {
                    num_used_pars = 4;
                  }
                }
                if (num_cmd_pars > 4)  // sweep number of extra time delays
                {
                  sw_extradelays = cmd_pars[4];
                  if ( (sw_extradelays < 0) || (sw_extradelays > max_sw_delays ) )
                  {
                    Serial.printf("ERROR: Invalid delay value for sweep (%d)\n",sw_extradelays);
                    invalid_cmd = 1;
                    break;
                  }
                  else
                  {
                    num_used_pars = 5;
                  }
                }
                sw_step = (sw_end - sw_start) / (sw_npts-1);
/**///                                                                      Serial.printf(".SWEEP %g to %g, %d pts, %d cycles, %d delays (step=%g)\n", sw_start, sw_end, sw_npts, sw_numcycles, sw_extradelays, sw_step);  
                
                //change relays if needed
                if ( px_prior_reed[n] == 0)
                {
/**///                                                                      Serial.printf("changing reed relay\n ");                    
                  digitalWrite(pins_reed[n],1);
                  px_reed[n] = 1;
                  sw_reedrelaychanged = 1;
                }
                if ( px_prior_ext[n] == 1)
                {
/**///                                                                      Serial.printf("changing ext relay\n ");                                      
                  digitalWrite(pins_ext[n],0);
                  sw_extrelaychanged = 1;
                }
                if ( sw_extrelaychanged || sw_reedrelaychanged )
                  delay(ee.relay_delay_ms);

                int loopcycles = sw_numcycles*2;
                if (loopcycles == 0)
                {
                  loopcycles = 1;
                }

                //start cyclic loop
                for (int c=0; c<loopcycles; c++)
                {
                  //perform sweep
                  uint8_t vpts = 0;
                  uint8_t cpts = 0;
                  double sw_maxpwr = -1000000;
                  for (int k=0; k<sw_npts; k++)
                  {
                    double nextVal;
                    if (c%2)
                      nextVal = sw_end - sw_step*k;
                    else
                      nextVal = sw_start + sw_step*k;
                    if (isbsw)
                    {
                      analogWrite(pins_dac[n], (v_com-nextVal)/dac_vstep );
                      delay(ee.bias_delay_ms * (1 + sw_extradelays) );
                      readADCsPartial(n);
                      if (adccomp) // overload
                      {
                        cpts++;
                      }
                      else
                      {
                        sw_volts[vpts] = v_com - adcvolts1 - cal_V1[n];
                        sw_curr[vpts] = (adcvolts2 - adcvolts1 - cal_diff[n] ) / res[n];
                        sw_pwr[vpts] = sw_volts[vpts] * sw_curr[vpts];
                        sw_maxpwr = max(sw_maxpwr, sw_pwr[vpts]);
                        sw_bias[vpts] = nextVal;
                        sw_times[vpts] = (millis() - tstart)/1000.0L;
                        vpts++;
                      }
                    }
                    else
                    {
                      voltageSeek(n, nextVal);
                      if (sw_extradelays)
                      {
                        delay(ee.bias_delay_ms*sw_extradelays);
                        voltageSeek(n, nextVal);
                      }
                      if (px_compl[n]) // overload
                      {
                        cpts++;
                      }
                      else
                      {
                        sw_volts[vpts] = volts[n];
                        sw_curr[vpts] = curr[n];
                        sw_pwr[vpts] = pwr[n];
                        sw_maxpwr = max(sw_maxpwr, sw_pwr[vpts]);
                        sw_bias[vpts] = bias[n];
                        sw_times[vpts] = (millis() - tstart)/1000.0L;
                        vpts++;
                      }
                    }
                  }
                  
                  //output data
                  if (loopcycles > 1)
                    Serial.printf("Cycle %d %s: ", c/2 + 1, (c%2)?"Backward":"Forward");
                  Serial.printf("Sweep results: %d/%d valid points, max power = %g\n", vpts, sw_npts, sw_maxpwr);
                  Serial.println("Volts   \tmA      \tmW      \tBias    \tTime (s)");
                  for (int k=0; k<vpts; k++)
                  {
                    Serial.printf("%12g\t%12g\t%12g\t%12g\t%12g\n", sw_volts[k], sw_curr[k]*1000, sw_pwr[k]*1000, sw_bias[k], sw_times[k]);
                  }
                  Serial.println("");

                }

                //restore old settings, delay if needed
                analogWrite(pins_dac[n], (v_com-px_prior_bias[n])/dac_vstep );
                delay(ee.bias_delay_ms);

                //change relays if needed
                if (sw_reedrelaychanged)
                {
/**///                                                                      Serial.printf("restoring reed relay\n ");                    
                  digitalWrite(pins_reed[n],px_prior_reed[n]);
                  px_reed[n] = px_prior_reed[n];
                }
                if ( sw_extrelaychanged == 1)
                {
/**///                                                                      Serial.printf("restoring ext relay\n ");                                      
                  digitalWrite(pins_ext[n],px_prior_ext[n]);
                }
                if ( sw_extrelaychanged || sw_reedrelaychanged )
                  delay(ee.relay_delay_ms);
                
                
              }
              else
              {
                Serial.printf("ERROR: Invalid pixel command \"%s\"\n", cmdstr);
                invalid_cmd = 1;
                break;
              }
            }
          }
          
          
          
         }
         //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         else ////////////////////////////////////////////////////////////////////////   SYSTEM COMMANDS //////////////////////////////////////////////
         {
         // Serial.print("System command: ");
         // Serial.println(cmdstr);
            if ( ( strcmp(cmdstr, "?") == 0 ) )                                               //////  ? (print)  //////////////////////////////////////
            {
              if ( printedHeader == 0 )
              {
                printHeader();
                printedHeader=1;
              }
              printOutputLine();
            }
            else if ( ( strcmp(cmdstr, "?HEAD") == 0 ) || ( strcmp(cmdstr, "?H") == 0 ) || ( strcmp(cmdstr, "?HEADER") == 0 ) )   
            {                                                                   /////  head?  (print table header) //////////////////////////////////////
                printHeader();
                printedHeader=1;
            }
            else if ( ( strcmp(cmdstr, "OI") == 0 ) || ( strcmp(cmdstr, "?OI") == 0 ) )                                                ///  OI (output interval)   /////////////////////////
            {
              if (cmdstr[0] == '?')
                Serial.printf("Output Interval: %d\n", skip_output_interval );
              else if (num_cmd_pars != 1)
              {
                Serial.println("ERROR:  you must specify the output interval with the OI command, e.g., \"OI 5\"");
                invalid_cmd = 1;
              }
              else
              {
                uint8_t newoi =  cmd_pars[0];
                skip_output_interval = newoi;
/**///  Serial.printf("Changed output interval to %d\n",newoi);
              }
            
            }
            else if ( ( strcmp(cmdstr, "SETTIME") == 0 ) || ( strcmp(cmdstr, "SETRTC") == 0 ) )        ////////////////////  Set time    ///////////////////////////////////////////////////
            {
/**///                                                                          Serial.print("SETTING RTC:  ");
                if (!( (num_cmd_pars == 5) || (num_cmd_pars == 6) ))
                {
                    Serial.println("ERROR:  Setting the RTC requires five or six parameters:  YEAR MONTH DAY HOUR MINUTE [SECONDS]");
                    Serial.println("        For example, \"setrtc 2020 12 4 13:20\" will set the RTC to December 4th 2020, 1:20 PM.");
                    invalid_cmd = 1;
                }
                else
                {
                  if (1)
                  {
                    //do stuff
                    //FIXME
                    Serial.println("FIXME");
                  }
                  num_used_pars = num_cmd_pars;
                }
            }
            else if ( ( strcmp(cmdstr, "?TIME") == 0 ) || ( strcmp(cmdstr, "?RTC") == 0 ) )        ////////////////////  Read time    ///////////////////////////////////////////////////
            {
              Serial.printf("The RTC time is %d-%d-%d %d:%02d:%02d\n", year(), month(), day(), hour(), minute(), second() );
            }
            else if ( ( strcmp(cmdstr, "USERTC") == 0 ) || ( strcmp(cmdstr, "USECLOCK") == 0 ) )    /////////////////  Use RTC in table /////////////////////////////////////////
            {
              if (useRTC) ;
              else
              {
                modechanged = 1;
                ee.auto_flags |= flag_ee_RTC;
              }
            }
            else if ( ( strcmp(cmdstr, "USESECONDS") == 0 ) || ( strcmp(cmdstr, "USEELAPSED") == 0 ) )   /////////////////  Use elapsed seconds in table /////////////////////////////////////////
            {
              if (useRTC) ;
              {
                modechanged = 1;
                ee.auto_flags &= ~flag_ee_RTC;
              }
            }
            else if ( ( strcmp(cmdstr, "SAVE") == 0 )  )                            /////////////////// SAVE:  save settings to EEPROM ////////////////////////////////////////////////
            {
              // save eeprom state now
              Serial.println("FIXME");
            }
            else if ( ( strcmp(cmdstr, "RESTORE") == 0 )  )                          ////////////////////  RESTORE:  load from eeprom   ///////////////////////////////////////////////////
            {
              // load eeprom state now
              Serial.println("FIXME");
            }
            else if ( ( strcmp(cmdstr, "TIC") == 0 )  )                              ////////////////////  TIC:  Reset elapsed time counter   ///////////////////////////////////////////////////
            {
              // reset elapsed time thingy
              tstart = millis();
            }


            else
            {
                Serial.printf("ERROR: Invalid system command \"%s\"\n", cmdstr);
                invalid_cmd = 1;
            }
            
         }

         if (invalid_cmd)
         {
              //(note:  error message has already been displayed!)
           //  Serial.printf("ERROR: Invalid pixel command or parameter value.  (%s)\n", cmdstr);
           //  break;
         }
         else 
         {
            
            byte relayschanged = 0;
            byte biaschanged = 0;

            for (int n=0; n<num_chans; n++)
            {
              if (px_prior_reed[n] != px_reed[n])
              { 
                relayschanged = 1; 
                break; 
              }
              if (px_prior_ext[n] != px_ext[n])
              {
                relayschanged = 1;
                break;
              }
            }
            for (int n=0; n<num_chans; n++)
            {
              if (px_prior_bias[n] != bias[n])
              { 
                biaschanged = 1; 
                break; 
              }
            }
            for (int n=0; n<num_chans; n++)
            {
              if (px_prior_mode[n] != px_mode[n])
              { 
                 modechanged = 1; 
                 break; 
              }
            }
            if (biaschanged)
            {
/**///                                                                      Serial.println("Bias delay");               
              delay(ee.bias_delay_ms);              
            }
            if (relayschanged)
            {
/**///                                                                      Serial.println("Relay delay");               
              delay(ee.relay_delay_ms);
            }
            if (modechanged)
            {
/**///                                                                      Serial.println("Mode changed");               
              printedHeader=0;
            } 
          }

          if (num_cmd_pars > num_used_pars )
              Serial.printf("WARNING:  Too many parameters provided for %s command.  Extra parameters ignored.\n", cmdstr);
                    

     }
     else
       Serial.println("Previous command ignored.");
     ser_pos = 0;
     ser_buff[0] = 0;
    }
    else  // incomplete command
    {
      if (ser_pos >= ser_buff_length)
      {
        if (inputValid) Serial.println("Error: command buffer exceeded.  Ignoring input.");
        inputValid = 0;
      }
      else
      {
        ser_buff[ser_pos++] = incomingByte;
        ser_buff[ser_pos] = '\0';  //keep our buffer terminated... not all parsing code pays attention to ser_pos.
        inputValid = 1;
      }
    }
  }
  if (SerialUSB.available() > 0) 
    return; // this makes us return to processing serial input, if more is present, before we proceed with the measurement/control stage.  
            // This is needed because some commands, e.g., an invalid one, will cause a break out of the serial input processing loop.
    


/*
  ////////temp
  Serial.println("Goal:  0.40V");
  voltageSeek(5, 0.40, ee.max_vsearch_step_bits, (uint16_t)((v_com-bias[5]) / dac_vstep));
  Serial.print("Result:\tV=");
  Serial.print(volts[5],4);
  Serial.print("\tI=");
  Serial.print(curr[5]*1000,3);
  Serial.print("\tP=");
  Serial.print(pwr[5]*1000,4);
  Serial.print("\tB=");
  Serial.println(bias[5],4);

  delay(500);
  
  
  return;

  */
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // read ADCs 
  ///////////////////////////////////////////////////////////////////////////////////////

  delay(1);
  tic = millis();
  
  byte hasMPPT = 0;
  byte hasSearch = 0;
  int act_chans = 0;
  for (byte n = 0; n < num_chans; n++)
  {
    mppt_hist[n][numiters % mppt_hist_depth] = 0;
    if (px_mode[n]) act_chans++;
    if (px_mode[n] && (px_mode[n] < pm_idle))
    {
      if (px_mode[n] == pm_mppt)
        hasMPPT = 1;
      if ( (px_mode[n]==pm_voc) || (px_mode[n]==pm_isc) || (px_mode[n]==pm_cv) || (px_mode[n]==pm_cc) )
        hasSearch = 1;
      readADCs(n);
      //px_compl[n] = adccomp;
      //volts[n] = v_com - adcvolts1 - cal_V1[n];
      //if (px_reed[n])
      //  curr[n] = (adcvolts2 - adcvolts1 - cal_diff[n] ) / res[n];
      //else
      //  curr[n] = 0;
      delay(ee.ch_delay_ms);
    }
    else
    {
      volts1[n] = 0;
      volts2[n] = 0;
      volts[n] = 0;
      curr[n] = 0;
    }
    pwr[n] = volts[n]*curr[n];
  }
  toc = millis();
  
  //Serial.print("   tmeas = ");
  //Serial.println((toc-tic));

  if (hasSearch)
  {
    for (byte n = 0; n < num_chans; n++ )
    {
      if (px_mode[n] == pm_voc) //VOC mode...  don't need to do any search, but let's update vtarg and itarg 
      {
        vtarg[n] = volts[n];
       // itarg[n] = 0;
      }
      else if (px_mode[n] == pm_isc ) // ISC mode.  Do voltage search to 0V:
      {
        voltageSeek(n, 0);
      }
      else if (px_mode[n] == pm_cv ) // constant volatge mode
      {
        voltageSeek(n, vtarg[n]);
        itarg[n] = curr[n];
      }
      else if (px_mode[n] == pm_cc ) // constant volatge mode
      {
        currentSeek(n, itarg[n]);
        vtarg[n] = volts[n];
      }
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  // MPPT code here
  /////////////////////////////////////////////////////////////////////////////////////////
  if (hasMPPT)
  {
    uint8_t step_bits;
    float hist_dir;
    float hist_nrm;
    for (byte n = 0; n < num_chans; n++ )
    {
      if (px_mode[n] == pm_mppt)
      {
        //figure out search step
        hist_dir = 0;
        for (int k=0;k<mppt_hist_depth; k++)
          hist_dir += mppt_hist[n][k];
        hist_nrm = hist_dir / ((float)mppt_hist_depth);
        hist_nrm = abs(hist_nrm);
          step_bits= ee.min_mppt_step_bits + hist_nrm * ((float)ee.fast_mppt_step_bits);

        double bias1 = bias[n] + ((1<<step_bits) * dac_vstep);
        double bias2 = bias[n] - ((1<<step_bits) * dac_vstep);
        if (bias[n] > 0)
        {
          biasH[n] = min(bias1, max_bias);
          biasL[n] = max(bias2, min_bias);
        }
        else
        {
          biasL[n] = min(bias1, max_bias);
          biasH[n] = max(bias2, min_bias);
        }  

        if ( (biasH[n] == max_bias) || (biasL[n] == max_bias) || (biasH[n] == min_bias) || (biasL[n] == min_bias) )
          px_compl[n] = 1;

        analogWrite(pins_dac[n], (v_com - biasH[n]) / dac_vstep );
      }
    }
    delay(ee.bias_delay_ms);

    //read high bias values
    for (byte n = 0; n < num_chans; n++)
    {
      if (px_mode[n] == 1)
      {
        /*adc1_vals[n] = 0;
        adc2_vals[n] = 0;
        adc->analogRead(pins_adc1[n]);
        adc->analogRead(pins_adc2[n]);
        for (byte m = 0; m < ee.num_avgs; m++)
        {  adc1_vals[n] += adc->analogRead(pins_adc1[n]);
        
        //for (byte m = 0; m < ee.num_avgs; m++)
          adc2_vals[n] += adc->analogRead(pins_adc2[n]);
        }
        volts1[n] = adc1_vals[n] * adc_vstep / ee.num_avgs;
        volts2[n] = adc2_vals[n] * adc_vstep / ee.num_avgs;*/
        readADCsPartial(n);
        voltsH[n] = v_com - adcvolts1 - cal_V1[n];
        currH[n] = (adcvolts2 - adcvolts1  - cal_diff[n]) / res[n];
        pwrH[n] = voltsH[n] * currH[n];
        px_compl[n] |= adccomp;
        delay(ee.ch_delay_ms);
      }
    }

    //apply low bias values
    for (byte n = 0; n < num_chans; n++)
    {
      if (px_mode[n] == 1)
      {
         analogWrite(pins_dac[n], (v_com - biasL[n]) / dac_vstep );
      }
    }
    delay(ee.bias_delay_ms);

    //read low bias values
    for (byte n = 0; n < num_chans; n++)
    {
      if (px_mode[n] == 1)
      {
        /*adc1_vals[n] = 0;
        adc2_vals[n] = 0;
        adc->analogRead(pins_adc1[n]);
        adc->analogRead(pins_adc2[n]);
        for (byte m = 0; m < ee.num_avgs; m++)
        {  adc1_vals[n] += adc->analogRead(pins_adc1[n]);
        
        
        //for (byte m = 0; m < ee.num_avgs; m++)
          adc2_vals[n] += adc->analogRead(pins_adc2[n]);
        }
        volts1[n] = adc1_vals[n] * adc_vstep / ee.num_avgs;
        volts2[n] = adc2_vals[n] * adc_vstep / ee.num_avgs;*/
        readADCsPartial(n);
        voltsL[n] = v_com - adcvolts1 - cal_V1[n];
        currL[n] = (adcvolts2 - adcvolts1 - cal_diff[n]) / res[n];
        pwrL[n] = voltsL[n] * currL[n];
        px_compl[n] |= adccomp;
        delay(ee.ch_delay_ms);
      }
    }
    

    for (byte n = 0; n < num_chans; n++)
    {
      if (px_mode[n] == 1)
      {
        if ( n == -1)   //debug printout
        {
          Serial.print("Ch ");
          Serial.print(n);
          Serial.print(" MPPT, step = ");
          Serial.print(step_bits);
          Serial.print("   hist = ");
          Serial.print(hist_dir);
          Serial.print("   tmeas = ");
          Serial.println((toc-tic));
          Serial.print("Bias      ");
            Serial.print(bias[n],5);
            Serial.print("\t");
            Serial.print(biasH[n],5);
            Serial.print("\t");
            Serial.println(biasL[n],5);
          Serial.print("Voltage   ");
            Serial.print(volts[n],5);
            Serial.print("\t");
            Serial.print(voltsH[n],5);
            Serial.print("\t");
            Serial.println(voltsL[n],5);
          Serial.print("Current   ");
            Serial.print(curr[n]*1000,5);
            Serial.print("\t");
            Serial.print(currH[n]*1000,5);
            Serial.print("\t");
            Serial.println(currL[n]*1000,5);
          Serial.print("Power     ");
            Serial.print(pwr[n]*1000,5);
            Serial.print("\t");
            Serial.print(pwrH[n]*1000,5);
            Serial.print("\t");
            Serial.println(pwrL[n]*1000,5);
          }

         if (pwrL[n] > pwr[n])
         {
           bias[n] = biasL[n];
           volts[n] = voltsL[n];
           pwr[n] = pwrL[n];
           curr[n] = currL[n];
           analogWrite(pins_dac[n], (v_com - biasL[n]) / dac_vstep );
           mppt_hist[n][numiters % mppt_hist_depth] = -1;
         }
         else if (pwrH[n] > pwr[n])
         {
           bias[n] = biasH[n];
           volts[n] = voltsH[n];
           pwr[n] = pwrH[n];
           curr[n] = currH[n];
           analogWrite(pins_dac[n], (v_com - biasH[n]) / dac_vstep );
           mppt_hist[n][numiters % mppt_hist_depth] = 1;
         }
         //else
         // mppt_hist[n][numiters % mppt_hist_depth] = 0;  redundant
         vtarg[n] = volts[n];
         itarg[n] = curr[n];
      }
    }
    delay(ee.bias_delay_ms);

  }

  tmppt = millis();
  
  if ( (skip_output_interval) && (numiters % skip_output_interval == 0) && act_chans )
  {
              if ( printedHeader == 0 )
              {
                printHeader();
                printedHeader=1;
              }
              printOutputLine();
  }
  numiters++;
  delay(ee.loop_delay_ms);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// printOutputLine() 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void printOutputLine()
{
  line_num++;
    Serial.print((int)numiters);
    Serial.print("\t");
    Serial.print( (tmppt - tstart) / 1000.0 , 3);
    Serial.print("\t\t");

    int act_chans = 0;
    for (int n = 0; n < num_chans; n++)
    {
      if (px_mode[n])
      {
        act_chans++;
        Serial.print(volts[n],4);
        Serial.print("\t");
        Serial.print(curr[n]*1000.0, 4);
        if (px_compl[n])
          Serial.print("\t!OVLD\t");
        else
          Serial.print("\t\t");
      }
    }
    if (!act_chans)
      Serial.print("No active channels");
      
    Serial.println("");
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// readADCs() 
// reads both ADCs for a given channel/pixel.  If possible it does it synchronously.  
// results are placed in global volts[], curr[], and pwr[] for that channel.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void readADCs(int chan) 
{
  /*uint32_t adc1_vals = 0;
  uint32_t adc2_vals = 0;
  adccomp = 0;

  if (pins_sync[chan] )
  {
     ADC::Sync_result asr;
     for (byte m = 0; m < ee.num_avgs; m++)
     {
       asr = adc->analogSyncRead(pins_adc1[chan], pins_adc2[chan]);
       adc1_vals += (uint16_t)asr.result_adc0;
       adc2_vals += (uint16_t)asr.result_adc1;
     }
  }
  else
  {
    adc->analogRead(pins_adc1[chan]);
    adc->analogRead(pins_adc2[chan]);
    for (byte m = 0; m < ee.num_avgs; m++)
    {  
      adc1_vals += adc->analogRead(pins_adc1[chan]);
      adc2_vals += adc->analogRead(pins_adc2[chan]);
    }
  }
  adcvolts1 = adc1_vals * adc_vstep / ee.num_avgs;
  adcvolts2 = adc2_vals * adc_vstep / ee.num_avgs;  

  int adc1_val = adc1_vals / ee.num_avgs;
  int adc2_val = adc2_vals / ee.num_avgs;

  if ( (adc1_val  < (1 << ee.min_mppt_step_bits) ) || (adc2_val < (1<<ee.min_mppt_step_bits) ) )
    adccomp = 1;
  else if ( (adc1_val >= ( (1<<adc_bits) - (1<<ee.min_mppt_step_bits)) ) || (adc2_val >= ( (1<<adc_bits) - (1<<ee.min_mppt_step_bits)) )  )
    adccomp = 1;*/

  readADCsPartial(chan);  

  px_compl[chan] = adccomp;
  volts[chan] = v_com - adcvolts1 - cal_V1[chan];
  if (px_reed[chan])
     curr[chan] = (adcvolts2 - adcvolts1 - cal_diff[chan] ) / res[chan];
  else
     curr[chan] = 0;
  pwr[chan] = volts[chan]*curr[chan];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// readADCsPartial() 
// reads both ADCs for a given channel/pixel.  If possible it does it synchronously.  
// results are placed in global variables adcvolts1 and adcvolts2 -- not to the volts[], curr[], and pwr[] arrays.  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void readADCsPartial(int chan) 
{
  uint32_t adc1_vals = 0;
  uint32_t adc2_vals = 0;
  adccomp = 0;

  if (pins_sync[chan] )
  {
     ADC::Sync_result asr;
     for (byte m = 0; m < ee.num_avgs; m++)
     {
       asr = adc->analogSyncRead(pins_adc1[chan], pins_adc2[chan]);
       adc1_vals += (uint16_t)asr.result_adc0;
       adc2_vals += (uint16_t)asr.result_adc1;
     }
  }
  else
  {
    adc->analogRead(pins_adc1[chan]);
    adc->analogRead(pins_adc2[chan]);
    for (byte m = 0; m < ee.num_avgs; m++)
    {  
      adc1_vals += adc->analogRead(pins_adc1[chan]);
      adc2_vals += adc->analogRead(pins_adc2[chan]);
    }
  }
  adcvolts1 = adc1_vals * adc_vstep / ee.num_avgs;
  adcvolts2 = adc2_vals * adc_vstep / ee.num_avgs;  

  int adc1_val = adc1_vals / ee.num_avgs;
  int adc2_val = adc2_vals / ee.num_avgs;

  if ( (adc1_val  < (1 << ee.min_mppt_step_bits) ) || (adc2_val < (1<<ee.min_mppt_step_bits) ) )
    adccomp = 1;
  else if ( (adc1_val >= ( (1<<adc_bits) - (1<<ee.min_mppt_step_bits)) ) || (adc2_val >= ( (1<<adc_bits) - (1<<ee.min_mppt_step_bits)) )  )
    adccomp = 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// voltageSeek
// recursive overshooting approach to a target voltage value, within one LSB of the DAC
//
// the entry function makes sure the starting "stepBits" value isn't too large, based on the current difference between the 
// target and actual voltage.  Then, we enter the recursive voltage seek function, which basically does a binary search to 
// find the correct bias value for the desired cell voltage.  The resulting operating voltage, current, pwr, and bias values
// are all updated within the global arrays. 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t voltageSeek(int chan, double goalV)
{
  readADCs(chan);
  uint16_t lastBias = (uint16_t)((v_com-bias[chan])/dac_vstep);
  uint8_t stepBits = ee.max_vsearch_step_bits;
  
  double deltav = abs(goalV - volts[chan]);
  uint16_t ratiov = deltav/dac_vstep;
  unsigned long vststart = millis();

  uint16_t step_size_limiter = 0;
  while ( ratiov )
  {
    ratiov = ratiov >> 1;
    step_size_limiter++;
  }
  stepBits = min(stepBits, step_size_limiter);
  uint16_t myresult = voltageSeekRec(chan, goalV, stepBits, lastBias );
//  Serial.printf("Voltage seek: %6g %6g  %d  (%d ms)\n", volts[chan], bias[chan], px_compl[chan], millis()-vststart );
  return myresult;
}

uint16_t voltageSeekRec(int chan, double goalV, int stepBits, long lastBias)
{
  /*if (  (lastBias <= 0) || (lastBias >= dac_max ) )
  {
    px_compl[chan] = 1;
    return lastBias;
  }*/  // don't break recursion yet, maybe we'll come back from compliance with a smaller step?
//Serial.print(stepBits);
  if ( (lastBias <= 10) || (lastBias >= dac_max-10) )
    px_compl[chan]=1;
    
  if (stepBits < 1) 
    return lastBias; // done!

  if (goalV == volts[chan])
    return lastBias; // done!

  int mydir = -1;
  if (goalV > volts[chan])
    mydir = 1;
    
  long nextBias = lastBias;
  while (   ( (mydir*goalV) > (mydir*volts[chan]) )    )
    {
      double somethingsBroken;
      nextBias = nextBias - (mydir*(1<<stepBits));
      if ( (nextBias < 0) || (nextBias > dac_max) )
      {
        nextBias += (mydir*(1<<stepBits));
        break;  // do not change bias, go back to previous value, then step down to next recursion.
      }
      //nextBias = max(nextBias, 0);
      //nextBias = min(nextBias, dac_max );
      somethingsBroken = v_com - double(nextBias)*dac_vstep;
      bias[chan] = v_com - double(nextBias)*dac_vstep;
      analogWrite(pins_dac[chan], nextBias );
      delay(ee.bias_delay_ms);
      readADCs(chan);
//     if (mydir > 0) Serial.print("^"); else Serial.print("v");
    }
/*
  Serial.print(nextBias);
  Serial.print("\t");
  Serial.print(bias[chan],4);
  Serial.print("\t");
  Serial.print(volts[chan],4);
  Serial.print("\t");
  Serial.print(stepBits);
  Serial.print("\t");
  Serial.println(mydir);
*/
  return voltageSeekRec( chan, goalV, stepBits-1, nextBias);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// currentSeek
// recursive overshooting approach to a target current value, within one LSB of the DAC
//
// derived from voltageSeek()
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long currentSeek(int chan, double goalI)
{
  readADCs(chan);
  long lastBias = (uint16_t)((v_com-bias[chan])/dac_vstep);
  int stepBits = ee.max_vsearch_step_bits;
  
  double deltai = abs(goalI - curr[chan]);
  double deltav = deltai*res[chan];
  long ratiov = deltav/dac_vstep;
  unsigned long iststart = millis();

  int step_size_limiter = 1;
  while ( ratiov )
  {
    ratiov = ratiov >> 1;
    step_size_limiter++;
  }
  stepBits = min(stepBits, step_size_limiter);
  long myresult = currentSeekRec(chan, goalI, stepBits, lastBias, 0 );
  Serial.printf("Current seek (%g mA): %6g mA, V=%6g, B=%6g, B0=%6g, %s  (%d ms)\n", goalI*1000, curr[chan]*1000 , volts[chan], bias[chan], v_com-(lastBias*dac_vstep), px_compl[chan]?("OVLD"):(""), millis()-iststart );
  return myresult;
}

long currentSeekRec(int chan, double goalI, int stepBits, long lastBias, int antihunt)
{
  /*if (  (lastBias <= 0) || (lastBias >= dac_max ) )
  {
    px_compl[chan] = 1;
    return lastBias;
  }*/  // don't break recursion yet, maybe we'll come back from compliance with a smaller step?
//Serial.printf("\n %12d %12g\n", lastBias, ((v_com-bias[chan])/dac_vstep) );

  if ( (lastBias <= 10) || (lastBias >= dac_max-10) )
    px_compl[chan]=1;
    
  if (stepBits <  1) 
    return lastBias; // done!
Serial.print(stepBits);
  if (goalI == curr[chan])
    return lastBias; // done!

  int mydir = -1;
  if (goalI < curr[chan])
    mydir = 1;
    
  long nextBias = lastBias;
  int stepCtr = 0;
  while (   ( (mydir*goalI) < (mydir*curr[chan]) )    )
    {
      double nextBiasDbl;
      if (stepCtr++ > 8)
      {
        if (antihunt <= ee.max_vsearch_step_bits/2 )
        {
          stepBits = min(stepBits+1, ee.max_vsearch_step_bits);
          Serial.print(stepBits);
          stepCtr=0;
          antihunt++;
        }
      }
      nextBias = nextBias - (mydir*(1<<stepBits));
      if ( (nextBias < 0) || (nextBias > dac_max) )
      {
        nextBias += (mydir*(1<<stepBits));
        break;  // do not change bias, go back to previous value, then step down to next recursion.
      }
      if (stepCtr++ > 10)
      //nextBias = max(nextBias, 0);
      //nextBias = min(nextBias, dac_max );
      nextBiasDbl = v_com - double(nextBias)*dac_vstep;
      bias[chan] = v_com - double(nextBias)*dac_vstep;
      //Serial.print(nextBiasDbl,8); Serial.print(" "); Serial.print(nextBiasDbl,8); Serial.print(" "); Serial.print(bias[chan],8); Serial.print(" "); Serial.println(bias[chan],8);
      if (mydir > 0) Serial.print("^"); else Serial.print("v");
      analogWrite(pins_dac[chan], nextBias );
     // Serial.printf("Set bias to %12g %12d %g %g \n",((v_com-bias[chan])/dac_vstep), nextBias, double(nextBias)*dac_vstep, bias[chan]);
      delay(ee.bias_delay_ms);
      readADCs(chan);

    }
/*
  Serial.print(nextBias);
  Serial.print("\t");
  Serial.print(bias[chan],4);
  Serial.print("\t");
  Serial.print(volts[chan],4);
  Serial.print("\t");
  Serial.print(stepBits);
  Serial.print("\t");
  Serial.println(mydir);
*/
  return currentSeekRec( chan, goalI, stepBits-1, nextBias, antihunt);
}

// didn't work right.  Off by four somehow...
/*uint16_t getBiasDAC(uint8_t chan)
{
  return analogRead(pins_dac[chan]) / ( 1<<( adc_bits - dac_bits) );
}*/ 


//not yet used, maybe useful?
void applyBias( uint8_t chan, double biasV)
{
    analogWrite(pins_dac[chan], (v_com-biasV)/dac_vstep );
    bias[chan] = biasV;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SELF CALIBRATION
// NOTE:  This doesn't seem to work at all!!!
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void self_cal()
{
  byte old_reed_state[num_chans];
  for (byte n = 0; n < num_chans; n++)
  {
    old_reed_state[n] = digitalRead(pins_reed[n]);
    digitalWrite(pins_reed[n],0);
  }
  delay(ee.relay_delay_ms);
  for (byte n = 0; n < num_chans; n++)
    analogWrite(pins_dac[n], v_com/dac_vstep );
  delay(ee.bias_delay_ms);

  for (byte n = 0; n < num_chans; n++)
  {
      /*adc1_vals[n] = 0;
      adc2_vals[n] = 0;
      adc->analogRead(pins_adc1[n]);
      adc->analogRead(pins_adc2[n]);
      for (byte m = 0; m < ee.num_avgs; m++)
      {
        adc1_vals[n] += adc->analogRead(pins_adc1[n]);
      
      //for (byte m = 0; m < ee.num_avgs; m++)
        adc2_vals[n] += adc->analogRead(pins_adc2[n]);
      }
      volts1[n] = adc1_vals[n] * adc_vstep / ee.num_avgs;
      volts2[n] = adc2_vals[n] * adc_vstep / ee.num_avgs;*/
      readADCs(n);
      cal_diff[n] = adcvolts2 - adcvolts1;
      cal_V1[n] = 0; //v_com-adcvolts1;
      Serial.print("CAL CH");
      Serial.print(n);
      Serial.print(" V1=");
      Serial.print(volts1[n],6);
      Serial.print(" V2=");
      Serial.print(volts2[n],6);
      Serial.println("");
  }
  cal_diff[0] = 0;
  cal_diff[1] = 0;
  for (byte n = 0; n < num_chans; n++)
    analogWrite(pins_dac[n], (v_com-bias[n])/dac_vstep );
  delay(10);
  for (byte n = 0; n < num_chans; n++)
    digitalWrite(pins_reed[n],old_reed_state[n]);
  delay(1000);

  Serial.println("CALIBRATION");
  Serial.print("CHAN ");
  for (byte n = 0; n < num_chans; n++)
  {
    Serial.print("\t");
    Serial.print(n);
  }
  Serial.println("");
  Serial.print("I0 (mA)");
  for (byte n = 0; n < num_chans; n++)
  {
    Serial.print("\t");
    Serial.print(cal_diff[n]*1000/res[n],3);
  }
  Serial.println("");
  Serial.print("V1 (mV)");
  for (byte n = 0; n < num_chans; n++)
  {
    Serial.print("\t");
    Serial.print(cal_V1[n]*1000.0,3);
  }
  Serial.println("");
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  processPxSpec()
//    this is going to SUCK!!!!
//    parses pixelspec strings as generously as possible.  Results wind up in px_sel[].  
//    returns index into remaining command string at which subsequent commands begin.  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int processPxSpec()
{
  // char ser_buff[ser_buff_length];
  //int ser_pos = 0;
  uint16_t pp = 0;
  byte inv=0;
  //byte rng=0;
  byte donewithpx=0;
  
  for (int k=0; k<num_chans; k++) px_sel[k] = 0;
  
  char numbuff[ser_buff_length];
  clrbuff(numbuff,ser_buff_length);
  uint16_t firstnum;
  uint16_t secondnum;
  
  String cmdstr = ser_buff;
  cmdstr = cmdstr.toUpperCase();
  cmdstr = cmdstr.trim();
  cmdstr.toCharArray(ser_buff,ser_pos+1);

  if (ser_buff[0] == 'P')  {                                 // pixelspec!
/**///                                                           Serial.println("P");
    pp=skipwhite(ser_buff,1);
/**///                                                            Serial.println(ser_buff[pp]);
    if ( (ser_buff[pp] == '*') || (ser_buff[pp] == '~') )
    {
      for (int k=0; k<num_chans; k++) px_sel[k] = 1;
      inv = (ser_buff[pp] == '~')? 1:0;                     /**///     if (ser_buff[pp] == '~' ) Serial.println("Inverting");
      donewithpx = (ser_buff[pp] == '*')? 1:0;              /**///     if (ser_buff[pp] == '*' ) Serial.println("*** encountered");
      pp++;
    }

    while (1)
    {
      pp=skipwhite(ser_buff,pp);                                                  

      if ( ser_buff[pp] == 'P' )
        pp++;

      pp=skipwhite(ser_buff,pp);                                            
/**///                                                                                Serial.print("Starting loop, next char: \""); Serial.print(ser_buff[pp]); Serial.println("\"");
      if ( (ser_buff[pp] == '*') || (ser_buff[pp] == '~') )
        return -8;
      
      int digs = 0;                                                                /**///Serial.println("Looking for first number");
      while( (ser_buff[pp] >= '0') && (ser_buff[pp] <= '9' ))
      {
        digs++;                                                                    /**///Serial.print(ser_buff[pp]);
        addtobuff(numbuff, ser_buff[pp++]);                                          
      }                                                                            /**///Serial.println("");
  
      if (digs <= 0)
        break; //return -2;  // no more digits...
      else
        firstnum = atol(numbuff);                                                  /**///Serial.print("Firstnum: "); Serial.println(firstnum); 

      if (donewithpx)
        return -2;
  
      if (firstnum >= num_chans)
        return -3;
  
      clrbuff(numbuff,ser_buff_length);
      digs = 0;
      pp=skipwhite(ser_buff,pp);                                                   /**///Serial.print("Next char: \""); Serial.print(ser_buff[pp]); Serial.println("\"");

      if ( (ser_buff[pp] == '*') || (ser_buff[pp] == '~') )
        return -9;
  
      if ( /*(ser_buff[pp] == ':') || */ (ser_buff[pp] == '-' ) ) // range mode
      {                                                                            /**///Serial.println("Range Mode");
        pp++;
        pp=skipwhite(ser_buff,pp);
        
        while( (ser_buff[pp] >= '0') && (ser_buff[pp] <= '9' ))
        {
          digs++;                                                                  /**///Serial.print(ser_buff[pp]);
          addtobuff(numbuff,ser_buff[pp++]);                                        
        }                                                                          /**///Serial.println("");
        if (digs <= 0)
          return -4;
        else
          secondnum = atol(numbuff);
        
        if (secondnum >= num_chans)
          return -5;
        if (secondnum <= firstnum )
          return -6;

        clrbuff(numbuff, ser_buff_length);
        digs = 0;
        pp=skipwhite(ser_buff,pp);
        
        for ( int l = firstnum; l <= secondnum; l++ )
        {
          px_sel[l] = !inv;
        }
      }
      else
      {
        px_sel[firstnum] = !inv;
      }

      if ( (ser_buff[pp] >= '0') && (ser_buff[pp] <= '9' ) )
        continue;

      if ( (ser_buff[pp] == ',')  || (ser_buff[pp] == 'P') )
      {                                                                               /**///Serial.println(",");                 
          pp++;
          continue;
      }
      else
        break;        
   }
   int tpx = 0;
   for (int i=0; i<num_chans; i++ )
   tpx += px_sel[i];

   if (tpx == 0)
    return -7;

  /* for (int i=0; i<num_chans; i++)
   { Serial.print(i); Serial.print("\t"); }
   Serial.println("");
   for (int i=0; i<num_chans; i++)
   { Serial.print(px_sel[i]); Serial.print("\t"); }
   Serial.println("");
   */
   return pp;
  }
  else
    return 0;
   
}

void clrbuff(char* numbuff, int sz)
{
  for (int i=0; i<sz; i++)
    numbuff[i]=0;
}

void addtobuff(char* numbuff, char digit)
{
  int i;
  for (i=0; numbuff[i] != 0; i++);
  numbuff[i] = digit;
}


uint16_t skipwhite(char* buff, uint16_t pos)
{
  while (isSpace(buff[pos]))
  {
    pos++;
  }
  return pos;
}

void printPx(uint8_t chan)
{
  if ( chan >= num_chans)
  {
    Serial.printf("ERROR: Channel %d doesn't exist!\n",chan);
    return;
  }

//  Serial.println("");
  Serial.printf("CHANNEL: %d\n",chan);
  Serial.printf("MODE: %s (%d)\n", modeStrs[px_mode[chan]], px_mode[chan] );
  if ((px_mode[chan] > pm_dis ) && (px_mode[chan] < pm_ext) )
  {
    Serial.printf("VOLTAGE: %g V %s\n", volts[chan],  px_compl[chan]?"OVERLOAD":"");
    Serial.printf("CURRENT: %g mA %s\n", curr[chan]*1000,  px_compl[chan]?"OVERLOAD":"");
    Serial.printf("BIAS: %g V\n", bias[chan]);
    Serial.printf("POWER: %g mW\n", pwr[chan]);
  } 
    
}


/*
uint16_t findNextToken(char* theBuff, uint16_t pos, char* tokens)
{
  for (uint16_t i = pos; theBuff[i] != 0; i++)
  {
    char mychar = theBuff[i];
    for (uint16_t j = 0; tokens[j] != 0; j++)
    {
      if ( theBuff[i] == tokens[j] )
        return i;
    }
  }
}

int findNextNonNumeral(char* buff, int pos)
{
  for (int i = pos; buff[i] != 0; i++)
  {
    if ( (buff[i] < '0') || (buff[i] > '9');
      break;
  }
  return i;
}*/






/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  EEPROM defaults go here!
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setupEEPROMDefaults()
{
  ee.num_avgs = 64;
  ee.file_num = 0;   // this is the NEXT safe file number.  Always increment on successful startup.
  ee.loop_delay_ms = 50;
  ee.bias_delay_ms = 2;
  ee.ch_delay_ms = 0;
  ee.relay_delay_ms = 100;
  ee.auto_flags = 0; // see flags above
  ee.min_mppt_step_bits = 2; // min dac step (x DAC LSB)
  ee.fast_mppt_step_bits = 3; // max dac step (x min_mppt_step_bits x DAC LSB)
  ee.max_vsearch_step_bits = 8;
  ee.unused = 0; // for the FUTURE
  ee.ch_enable = (1 << (num_chans+1)) - 1;
  ee.px_ressel = 0;
  for (int m=0; m<num_chans; m++)
  {
    ee.px_res1[m] = 101.0;
    ee.px_res2[m] = 10.0;
    ee.px_cal1[m] = 0.0;
    ee.px_cal2[m] = 0.0;
   // ee.px_ressel[m] = 0;
    ee.px_mode[m] = 0;
  }
//  ee.px_res1 = { 101.0, 101.0, 101.0, 101.0, 101.0, 101.0, 101.0, 101.0, 101.0, 101.0, 101.0, 101.0 };
// ee.px_res2 = { 10.0,  10.0,  10.0,  10.0,  10.0,  10.0,  10.0,  10.0,  10.0,  10.0,  10.0,  10.0  };
//  ee.px_cal1 = {  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0  };
// ee.px_cal2 = {  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0  };
}
