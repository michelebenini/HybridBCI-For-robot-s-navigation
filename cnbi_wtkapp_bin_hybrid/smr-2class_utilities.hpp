#ifndef SMR_2CLASS_UTILITIES_HPP
#define SMR_2CLASS_UTILITIES_HPP

#include <unistd.h>

#include <wtkdraw/Bar.hpp>
#include <wtkdraw/Cross.hpp>
#include <wtkdraw/Arrow.hpp>
#include <wtkdraw/Circle.hpp>
#include <wtknet/ClTobiIc.hpp>
#include <wtknet/ClTobiId.hpp>
#include <wtkcore/Time.hpp>

#include <vector>
#include <drawtk.h>

#define SMR_2CLASS_WIN_WIDTH	  1024
#define SMR_2CLASS_WIN_HEIGHT 	768

#define SMR_2CLASS_BAR_WIDTH 	  0.3f
#define SMR_2CLASS_BAR_HEIGHT 	0.6f
#define SMR_2CLASS_BAR_THICK 	  0.02f

#define SMR_2CLASS_FIX_SIZE	    0.2f
#define SMR_2CLASS_FIX_THICK  	0.03f

#define SMR_2CLASS_CUE_SIZE  	  0.1f

#define SMR_2CLASS_AMPLITUDE 	  0.6f

/*#define SMR_2CLASS_XML_EVENTS 	"/home/stefano/.whitk/xml/wtk_events.xml"
#define SMR_2CLASS_XML_TASKS	  "/home/stefano/.whitk/xml/wtk_tasks.xml"
#define SMR_2CLASS_XML_LOOP	    "/home/stefano/.whitk/xml/wtk_smr_loop.xml"*/

#define SMR_2CLASS_XML_EVENTS 	"/home/michele/.whitk/xml/wtk_events.xml"
#define SMR_2CLASS_XML_TASKS	  "/home/michele/.whitk/xml/wtk_tasks.xml"
#define SMR_2CLASS_XML_LOOP	    "/home/michele/.whitk/xml/wtk_smr_loop.xml"

using namespace wtk::draw;

typedef Circle  Cue;
typedef Cross   Fixation;
typedef std::vector<Bar*> Bars;
const std::vector<const float*> ColorSet = {dtk_plum_dark, dtk_chameleon_med, dtk_orange_med};

typedef wtk::core::Container<std::string, std::string> Options;
typedef wtk::core::Container<std::string, unsigned int> ClassMap;

std::string xml_events;
std::string xml_tasks;
std::string xml_bci;


void completePath(const std::string relpath, std::string& fullpath){
	std::string homedir(getenv("HOME"));
	fullpath = homedir + relpath;
};

void completeAllPath(){
	completePath(SMR_2CLASS_XML_EVENTS, xml_events);
	completePath(SMR_2CLASS_XML_TASKS, xml_tasks);
	completePath(SMR_2CLASS_XML_LOOP, xml_bci);

};

void smr_2class_setup_bars_offline(Bars& bars) {

  bars.at(0)->Move(-0.75f, 0.15f);
  bars.at(0)->SetColor(ColorSet.at(0));
  bars.at(0)->SetAlpha(0.5f, Shape::Fill);
  bars.at(0)->SetValue(0.0f);
	
  bars.at(1)->Move(0.75f, 0.15f);
	bars.at(1)->SetColor(ColorSet.at(1));
	bars.at(1)->SetAlpha(0.5f, Shape::Fill);
	bars.at(1)->SetValue(0.0f);

  bars.at(2)->Move(0.0f, 0.155f);
  bars.at(2)->SetColor(ColorSet.at(2));
  bars.at(2)->SetAlpha(0.5f, Shape::Fill);
  bars.at(2)->SetValue(0.0f);
}

void smr_2class_setup_fixation_offline(Fixation* fixation) {
  fixation->Move(0.0f, -0.20f);
	fixation->SetColor(dtk_white);
}

void smr_2class_setup_cue_offline(Cue* cue) {
  cue->Move(0.0f, -0.20f);
	cue->SetColor(dtk_white);
	cue->SetAlpha(0.0f);
}

void smr_2class_setup_bars_online(Bars& bars) {

  bars.at(0)->Move(-0.5f, 0.0f);
  bars.at(0)->SetColor(ColorSet.at(0));
  bars.at(0)->SetAlpha(0.5f, Shape::Fill);
  bars.at(0)->SetValue(0.0f);

  bars.at(1)->Move(0.5f, 0.0f);
  bars.at(1)->SetColor(ColorSet.at(1));
  bars.at(1)->SetAlpha(0.5f, Shape::Fill);
  bars.at(1)->SetValue(0.0f);
}

void smr_2class_setup_fixation_online(Fixation* fixation) {
  fixation->Move(0.0f, 0.0f);
  fixation->SetColor(dtk_white);
}

void smr_2class_setup_cue_online(Cue* cue) {
  cue->Move(0.0f, 0.0f);
  cue->SetColor(dtk_white);
  cue->SetAlpha(0.0f);
}

void smr_2class_update_bars(Bars& bars, float value, unsigned int bId) {
	
  float cvalue[3] = {0.0f, 0.0f, 0.0f};
	cvalue[bId] = std::abs(value);

	bars.at(0)->SetValue(cvalue[0]);
	bars.at(1)->SetValue(cvalue[1]);
  bars.at(2)->SetValue(cvalue[2]);
}

float smr_2class_normalize_pp(float value, float max, float min) {
	float nvalue;
	nvalue = ((1.0f - 0.0f) * (value - min))/(max - min) + 0.0f;
	nvalue = nvalue >= 1.0f ? 1.0f : nvalue;
	nvalue = nvalue < 0.0f ? 0.0f : nvalue;
	return nvalue;
}


bool smr_2class_getopts(Options* opt, int narg, char** varg) {

	completeAllPath();
	
	int c;
	bool result = false;
	while ((c = getopt(narg, varg, "x:e:f")) != -1) {
		switch(c) {
			case 'x':
				opt->Set("xml-protocol", optarg);
				result = true;
				break;
			case 'e':
				opt->Set("xml-events", optarg);
				break;
			case 'f':
				opt->Set("win-w", "0");
				opt->Set("win-h", "0");
				break;
			default:
				break;
		}
	}

    //opt->Set("xml-tasks",  SMR_2CLASS_XML_TASKS);
    //opt->Set("xml-events", SMR_2CLASS_XML_EVENTS);
    opt->Set("xml-tasks",  xml_tasks);
    opt->Set("xml-events", xml_events);
    opt->Set("win-w",      std::to_string(SMR_2CLASS_WIN_WIDTH));
    opt->Set("win-h",      std::to_string(SMR_2CLASS_WIN_HEIGHT));

	return result;
};



#endif
