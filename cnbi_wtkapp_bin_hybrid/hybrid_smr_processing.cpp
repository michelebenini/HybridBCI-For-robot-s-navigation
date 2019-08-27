#ifndef SMR_2CLASS_PROCESSING_CPP
#define SMR_2CLASS_PROCESSING_CPP

#include <vector>

#include <cnbiloop/ClLoop.hpp>
#include <wtkprotocol/TaskSet.hpp>

#include <wtkcore/Time.hpp>
#include <wtknet/IFaceC.hpp>
#include <wtknet/IFaceD.hpp>

#include "../../src/SmrBci.hpp"
#include "../../src/Debug.hpp"
#include "../../src/wtk_app_utilities.hpp"
#include "smr-2class_utilities.hpp"

using namespace std;
using namespace Eigen;
using namespace wtk::core;
using namespace wtk::app;
using namespace wtk::protocol;
using namespace wtk::net;

void usage(void) {
  printf("\nUsage: smr-2class_processing -x XML_PROTOCOL [OPTIONS]...\n");
	printf("\n Options:");
	printf("\n  -p	PIPENAME 	iC pipe name				 [Default: '/ctrl0']");
	printf("\n  -b	BUSNAME		iD bus name 				 [Default: '/bus']");
	printf("\n  -e	XML_EVENTS	Path to xml file with events 		 [Default: $HOME/.whitk/xml/wtk_events.xml]");
	printf("\n  -l	XML_LOOP	Path to xml file with loop configuration [Default: $HOME/.whitk/xml/wtk_smr_loop.xml]");
	printf("\n  -d  		Export txt file with probabilities 	 [Default: False]");
}

int main(int argc, char* argv[]) {
	CcLogInfo("Start hybrid smr processing");
  	completeAllPath();
	int opt;
	std::string xmlprotocol;
  	std::string xmlbci    = SMR_2CLASS_XML_LOOP;
  	std::string xmlevents = SMR_2CLASS_XML_EVENTS;
  //std::string xmlbci    = xml_bci;
  //std::string xmlevents = xml_events;
	std::string icpipe  = "/ctrl0";
	std::string idpipe  = "/bus";
	bool optdebug  = true;
	int count = 0;


	while((opt = getopt(argc, argv, "x:e:l:p:b:d")) != -1) {
		switch(opt) {
			case 'x':
				xmlprotocol.assign(optarg);
				break;
			case 'e':
				xmlevents.assign(optarg);
				break;
			case 'l':
				xmlbci.assign(optarg);
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
	

	SmrBci* 	bci;
	IFaceC  	ic(IFaceC::AsSender);
	IFaceD  	id(IFaceD::AsReceiver);
	TaskSet 	taskset("smr");	
	float 		integration;
	float 		rejection;
	Probability 	pp;
	unsigned int 	idevent;
	TimeValue 	acqsync;
	string 		classifier;
	float 		elapsed = 0.0f;
	
	CcLogInfo("Configuring TaskSet");
	/***** Configuring taskset *****/
	if(taskset.Configure(xmlprotocol, xmlevents) == 0) {
		CcCore::CloseLogger();
		Core::Exit(0);
	}
	
	/***** Configuring classifier *****/
	try {	
		classifier = wtk_get_xmlfield(xmlprotocol, "wtkprotocol", "classifiers/smr");
	} catch (std::runtime_error& e) {
		CcLogFatalS(e.what());
		CcCore::CloseLogger();
		Core::Exit(0);
	}
	
	/***** CNBI Loop setup *****/
  CcCore::OpenLogger("smr-2class_processing");
	ClLoop::Configure();
	if(ClLoop::Connect() == false) {
		CcLogFatal("Cannot connect to loop");
		CcCore::CloseLogger();
		Core::Exit(0);
	}	
	
  /***** BCI SMR setup *****/
  CcLogInfo("Configuring BCI");

  try {
    bci = new SmrBci(&taskset, xmlbci);
    CcLogInfo("Smr Bci correctly configured, reading from pipe");
	} catch (std::runtime_error& e) {
		CcLogFatalS(e.what());
		CcCore::CloseLogger();
		Core::Exit(0);
	}

	////////////////////////////// ERROR
	
	bci->Dump();
	
	try {
		bci->Setup(classifier);
		CcLogInfo("Smr Bci correctly setup");
	} catch (std::runtime_error& e) {
		CcLogFatalS(e.what());
		CcCore::CloseLogger();
		Core::Exit(0);
	}


	try {
		rejection   = stof(wtk_get_xmlfield(xmlprotocol, "wtkprotocol", "rejection"));
		integration = stof(wtk_get_xmlfield(xmlprotocol, "wtkprotocol", "integration"));
		bci->SetRejection(rejection);
		bci->SetIntegration(integration);
	} catch (std::runtime_error& e) { CcLogWarningS(e.what() << " Using default value"); }
	
	/***** Configure Interface C ****/
    ic.Set("smr-2class", "");
	try {
		ic.CreateMessage(&taskset);
	} catch (std::runtime_error& e) {
		CcLogFatalS(e.what());
		CcCore::CloseLogger();
		Core::Exit(0);
	}
	ic.Dump();
	
	/***** Configure Interface D ****/
    id.Set("smr-2class");
	id.Dump();

	/**** Try to attach to interfaces ****/
	ic.TryAttach(icpipe, true);
	id.TryAttach(idpipe, true);

	CcLogInfo("Running SMR classification");
	Time::Tic(&acqsync);
	while(true) {
		
		if(ic.IsAttached() == false) {
			CcLogFatal("Interface C disconnected");
			break;
		}
		
		if(id.IsAttached() == false) {
			CcLogFatal("Interface D disconnected");
			break;
		}
	
		elapsed = Time::Toc(&acqsync);
		Time::Tic(&acqsync);
		
			
		if(bci->Classify() == false) {
			continue;
		}
		
		if(id.HasEvent()) {
			idevent = id.GetEvent();
			if(idevent == 781) {
				bci->Reset();
				CcLogInfoS("Reset event: NDF=" << bci->GetFrameIdx() << "/Evt=" << idevent);
			}
		}

		pp = bci->ipp;

		for(auto it = pp.Begin(); it != pp.End(); it++)
			ic.SetValue(it->first, it->second);

		ic.Send(bci->GetFrameIdx());

		if(elapsed >= 1.5f*bci->GetFrameRate()) 
			CcLogWarningS("Running late: " << elapsed/1000.0f << " seconds");
	}
	
	CcLogInfo("Closing processing");
	
	Time::Sleep(1000);

	CcCore::CloseLogger();
	Core::Exit(0);

}

#endif
