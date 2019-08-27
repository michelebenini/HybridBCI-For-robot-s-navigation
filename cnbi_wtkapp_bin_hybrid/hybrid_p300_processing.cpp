#ifndef HYBRID_P300_CPP
#define HYBRID_P300_CPP

#include <vector>

#include <cnbiloop/ClLoop.hpp>
#include <wtkprotocol/TaskSet.hpp>

#include <wtkcore/Time.hpp>
#include <wtknet/IFaceC.hpp>
#include <wtknet/IFaceD.hpp>

#include "../../src/p300BciNew.hpp"
#include "../../src/Debug.hpp"
#include "../../src/wtk_app_utilities.hpp"
#include "p300_utilities.hpp"
#include <wtkprotocol/Timings.hpp>
#include <wtkprotocol/Events.hpp>

using namespace std;
using namespace Eigen;
using namespace wtk::core;
using namespace wtk::app;
using namespace wtk::protocol;
using namespace wtk::net;


void usage(void) {
  printf("\nUsage: p300_processing -x XML_PROTOCOL [OPTIONS]...\n");
  printf("\n Options:");
  printf("\n  -p    PIPENAME    iC pipe name                 [Default: '/ctrl0']");
  printf("\n  -b    BUSNAME     iD bus name                  [Default: '/bus']");
  printf("\n  -e    XML_EVENTS  Path to xml file with events         [Default: $HOME/.whitk/xml/wtk_events.xml]");
  printf("\n  -l    XML_LOOP    Path to xml file with loop configuration [Default: $HOME/.whitk/xml/wtk_p300_loop.xml]");
  printf("\n  -d        Export txt file with probabilities   [Default: False]\n");
}


int main(int argc, char* argv[]) {
  CcLogInfo("Start hybrid p300 processing");
  completeAllPath(); // EVENTUALLY TO REMOVE

  int opt;
  std::string xmlprotocol;
  std::string icpipe  = "/ctrl0";
  std::string idpipe  = "/bus";
  float threshold = 0.0f;
  bool prediction = false;
  string predicted_image ="";
  bool optdebug  = true;


  CcLogInfo("Watching arguments");
  while((opt = getopt(argc, argv, "x:e:l:p:b:d")) != -1) {
    switch(opt) {
      case 'x':
        xmlprotocol.assign(optarg);
        break;
      case 'e':
        xml_events.assign(optarg);
        break;
      case 'l':
        xml_bci.assign(optarg);
        break;
      case 'p':
        icpipe.assign(optarg);
        break;
      case 'b':
        idpipe.assign(optarg);
        break;
      case 'd':
        optdebug = true;
        break;
  break;
      default:
        usage();
        return -1;
    }
  }

  CcLogInfo("Initialize BCI");
  p300BciNew* bci;
  IFaceD    id(IFaceD::AsReceiver);
  IFaceD    id1(IFaceD::AsSender);
  Events  events;
  Timings timings;
  TaskSet   taskset("p300");
  
  /*  Probability   pp; */
  unsigned int  idevent;
  //TimeValue   acqsync;
  string        classifier;
  int       target = -1;
  int       trigger = -1;
  bool navigation_active = false;
  unsigned int classindex;
  int       startTrial = 0; //number of trials that could start before elapsed
  //float       elapsed = 0.0f;

  /***** Configuring taskset *****/
  CcLogInfo("Configuring TaskSet");
 if(taskset.Configure(xmlprotocol, xml_events) == 0) {
    CcCore::CloseLogger();
    Core::Exit(0);
  }

  /***** Configuring timings *****/
  CcLogInfo("Configuring timings");
  if(timings.Configure(xmlprotocol) == 0) {
    CcCore::CloseLogger();
    Core::Exit(0);
  }

 /***** Configuring events *****/
  CcLogInfo("Configuring events");
 if(events.Configure(xml_events) == 0) {
   CcCore::CloseLogger();
   Core::Exit(0);
 }

  CcLogInfo("Configuring buffers");
  int  nbuffer = ceil(EPOCH_SIZE/(timings.Get("flash") + timings.Get("interflash"))); // number of  overlapping flashes
  TimeValue       acqsync[nbuffer];
  int             idxArray[nbuffer];  // ={};
  float           elapsed[nbuffer]; //={};

  std::fill(idxArray, idxArray+nbuffer, -1);


  /***** Configuring classifier *****/
  CcLogInfo("Configuring classifier");
  try {
    classifier = wtk_get_xmlfield(xmlprotocol, "wtkprotocol", "classifiers/p300");
  } catch (std::runtime_error& e) {
    CcLogFatalS(e.what());
    CcCore::CloseLogger();
    Core::Exit(0);
  }
  

  /***** CNBI Loop setup *****/
 CcCore::OpenLogger("CNBI Loop setup");
 ClLoop::Configure();
 if(ClLoop::Connect() == false) {
    CcLogFatal("Cannot connect to loop");
    CcCore::CloseLogger();
    Core::Exit(0);
  }


  /***** BCI P300 setup *****/
  CcLogInfo("BCI P300 setup");
  try {
    bci = new p300BciNew(&taskset, xml_bci);
    //bci = std::make_shared<p300Bci>(&taskset,xmlbci);
    CcLogInfo("p300 Bci correctly configured, reading from pipe");
  } catch (std::runtime_error& e) {
    CcLogFatalS(e.what());
    CcCore::CloseLogger();
    Core::Exit(0);
 }

 bci->Dump();

try {
    bci->Setup(classifier);
    CcLogInfo("P300 Bci correctly setup");
  } catch (std::runtime_error& e) {
    CcLogFatalS(e.what());
    CcCore::CloseLogger();
    Core::Exit(0);
  }

  /***** Configure Interface D ****/
 id.Set("P300");
 id.Dump();
 id1.Set("P300");
 id1.Dump();

  /**** Try to attach to interfaces ****/
  id.TryAttach(idpipe, false);
  id1.TryAttach(idpipe, false);

  CcLogInfo("Running p300 classification");
  int while_count = 0;
  while(true) {
    
    if(id.IsAttached() == false) {
      CcLogFatal("Interface D disconnected");
      break;
    }

    if(id1.IsAttached() == false) {
      CcLogFatal("Interface D disconnected");
      break;
    }

    bci->FillData();

    // guardo se c'è un evento
    if(id.HasEvent()) {
      
      // prendo il codice evento
      idevent = id.GetEvent();
      
      
      // p300_task1 = 0x0201 = 513 e successivi 
      if(idevent == events.Get("p300_task1")) { target = 0; std::cout<< "----     target 1    ----" << std::endl; }
      if(idevent == events.Get("p300_task2")) { target = 1; std::cout<< "----     target 2    ----" << std::endl; }
      if(idevent == events.Get("p300_task3")) { target = 2; std::cout<< "----     target 3    ----" << std::endl; }
      if(idevent == events.Get("p300_task4")) { target = 3; std::cout<< "----     target 4    ----" << std::endl; }
      
      // se non è relativo ad una p300 generata
      if(target ==-1 )
      {
        // guardo se è relativo ad un flash avvenuto
        // flash = 0x0500 = 1280 --> flash img 1 = 513 + 1280 = 1793
        if(idevent == events.Get("flash") + events.Get("p300_task1")) { classindex = 0; trigger = 1; std::cout<< "flash first image" << std::endl; }
        if(idevent == events.Get("flash") + events.Get("p300_task2")) { classindex = 1; trigger = 1; std::cout<< "flash second image" << std::endl; }
        if(idevent == events.Get("flash") + events.Get("p300_task3")) { classindex = 2; trigger = 1; std::cout<< "flash third image" << std::endl; }
        if(idevent == events.Get("flash") + events.Get("p300_task4")) { classindex = 3; trigger = 1; std::cout<< "flash fourth image" << std::endl; }

        // se c'è stato un flash ne tengo traccia
        if(trigger!=-1)
        {

            Time::Tic(&acqsync[startTrial]);
            idxArray[startTrial] = classindex;
            trigger=-1;
            startTrial ++;
            if (startTrial >= nbuffer) { startTrial = 0; }
            //std::cout <<"DEBUG startTrial: " << startTrial << std::endl;
        }

        // se il codice è: fine tentativo (attempt off)
        // attemp = 0x0700 = 1792 e off = 0x8000 = 32768 da sommare = 34560
        if(idevent ==  events.Get("attempt") +  events.Get("off") )
        {
            std::cout<< std::endl << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!                  end trial                   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            unsigned int predicted_class = bci->Predict();
            std::cout << " Verifica coefficient " << bci->GetProbability(predicted_class) << std::endl;
            
            // se la probabilità del risultato è maggiore di una soglia
            if(bci->GetProbability(predicted_class) > threshold)
            {  std::cout<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!                  PREDICTION                 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "  << std::endl; //<<  predicted_class  // the prediction about this trial
                // salvo la classe predetta
                predicted_image = "p300_task" + to_string(predicted_class + 1);
                prediction =true;
            }
            else
            {
                predicted_image = "unknown";
            }

            std::cout << " Predicted image: " << predicted_image << std::endl;
            
            // invio la classe predetta
            id1.Send(events.Get("prediction") + events.Get(predicted_image));
            while(id.HasEvent()){}
            
            // invio la fine predizione 
            // prediction = 0x0600 = 1536 + p300_taskX/unknown(0x0207) + off
            // prediction unknown = 1536 + 519 + 32768
            id1.Send(events.Get("prediction") + events.Get(predicted_image) + events.Get("off"));

            startTrial = 0;
            std::fill(idxArray, idxArray+nbuffer, -1);
            std::fill(elapsed, elapsed+nbuffer, 0);

            // se la predizione è stata fatta
            if(prediction)
            {
                // resetto la BCI
                std::cout << " RESET BCI " << std::endl;
                target = -1;
                navigation_active = false;
                bci->Reset();
            }

        }

      }


    }
    // se c'è stato un evento ma il target è rimasto -1 azzero i buffer
    if(target!=-1)
    {
        //std::cout <<"DEBUG - NBUFFER " << nbuffer << std::endl;
      for (int i=0; i<nbuffer; i++)
      {
        if(idxArray[i] != -1) {

          elapsed[i] = Time::Toc(&acqsync[i]);
          //std::cout<<"ELAPSED-DEBUG !!! " << elapsed[i] << std::endl;
          if(elapsed[i] >= (EPOCH_SIZE)*1000) //convert EPOCH_SIZE from seconds to milliseconds
          {
              std::cout << "CLASSIFYING" << std::endl;
              bci->Classify(idxArray[i]);
              elapsed[i] = 0.0f;
              idxArray[i] = -1;
          }
        }
      }
    }
  }

  CcLogInfo("Closing hybrid p300 processing");

  Time::Sleep(1000);

  CcCore::CloseLogger();

  Core::Exit(0);

}

#endif


