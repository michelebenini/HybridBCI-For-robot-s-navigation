#ifndef P300_UTILITIES_HPP
#define P300_UTILITIES_HPP

#include <unistd.h>

#include <wtkdraw/Image.hpp>
#include <wtknet/ClTobiIc.hpp>
#include <wtknet/ClTobiId.hpp>
#include <wtkcore/Time.hpp>

#include <vector>
#include <drawtk.h>
#include <string>

#define P300_WIN_WIDTH	  1024
#define P300_WIN_HEIGHT 	768

#define P300_IMAGE_WIDTH 	  0.5f
#define P300_IMAGE_HEIGHT 	0.5f
#define P300_IMAGE_THICK 	  0.015f
#define P300_IMAGE_MIPMAP_MAXLEVEL 1

#define P300_CUE_WIDTH 	  0.4f
#define P300_CUE_HEIGHT 	0.4f
#define P300_CUE_THICK 	  0.0f

#define EPOCH_SIZE  0.6f   //1.0f
#define DOWNSAMPLING 8


#define P300_XML_EVENTS 	"/.whitk/xml/wtk_events.xml"
#define P300_XML_TASKS	  "/.whitk/xml/wtk_tasks.xml"				// EVENTUALLY TO REMOVE	
#define P300_XML_LOOP	    "/.whitk/xml/wtk_p300_loop.xml"

//#define P300_TASK_IMAGE1  "/.whitk/img/micky.jpg"
//#define P300_TASK_IMAGE2  "/.whitk/img/donald.jpg"
//#define P300_TASK_IMAGE3  "/.whitk/img/goofy.jpg"
//#define P300_TASK_IMAGE4  "/.whitk/img/pluto.jpg"

#define P300_TASK_IMAGE1  "/.whitk/img/micky_transparent.png"
#define P300_TASK_IMAGE2  "/.whitk/img/donald_transparent.png"
#define P300_TASK_IMAGE3  "/.whitk/img/goofy_transparent.png"
#define P300_TASK_IMAGE4  "/.whitk/img/pluto_transparent.png"


#define DATA     "/workspace/whitoolkit/data/P300/"

using namespace wtk::draw;

typedef Image Cue;
typedef std::vector<Image*> Images;

std::vector<const char*> ImageSet;

typedef wtk::core::Container<std::string, std::string> Options;
typedef wtk::core::Container<std::string, unsigned int> ClassMap;

std::string task_image1;
std::string task_image2;
std::string task_image3;
std::string task_image4;

std::string xml_events;
std::string xml_tasks;
std::string xml_bci;
std::string xml_data;


void completePath(const std::string relpath, std::string& fullpath){
	std::string homedir(getenv("HOME"));
	fullpath = homedir + relpath;
};

void fillImageSet(){	
	const char* p300_task_image1;
	const char* p300_task_image2;
	const char* p300_task_image3;
	const char* p300_task_image4;

	p300_task_image1 = task_image1.c_str();
	p300_task_image2 = task_image2.c_str();
	p300_task_image3 = task_image3.c_str();
	p300_task_image4 = task_image4.c_str();

	ImageSet = {p300_task_image1,
				p300_task_image2,
				p300_task_image3,
				p300_task_image4};
};

void completeAllPath(){
	completePath(P300_XML_EVENTS, xml_events);
	completePath(P300_XML_TASKS, xml_tasks);
	completePath(P300_XML_LOOP, xml_bci);
	completePath(DATA, xml_data);

	completePath(P300_TASK_IMAGE1, task_image1);
	completePath(P300_TASK_IMAGE2, task_image2);
	completePath(P300_TASK_IMAGE3, task_image3);
	completePath(P300_TASK_IMAGE4, task_image4);

	fillImageSet();

};

void p300_setup_images_offline(Images& images) {

  images.at(0)->Move(-0.70f, 0.00f);
  images.at(0)->SetAlpha(0.10f, Shape::Fill);
	
  images.at(1)->Move(0.00f, 0.70f);
  images.at(1)->SetAlpha(0.10f, Shape::Fill);

  images.at(2)->Move(0.70f, 0.00f);
  images.at(2)->SetAlpha(0.10f, Shape::Fill);

  images.at(3)->Move(0.00f, -0.70f);
  images.at(3)->SetAlpha(0.10f, Shape::Fill);

}

void p300_setup_cue_offline(Cue* cue) {
  cue->Move(0.0f, 0.00f);
  cue->SetAlpha(0.00f, Shape::Fill);
}


void p300_flash_images(Images& images, unsigned int imageId) {
	
  images.at(imageId)->SetAlpha(1.0f, Shape::Fill);
}


void p300_unflash_images(Images& images, unsigned int imageId) {

  images.at(imageId)->SetAlpha(0.10f, Shape::Fill);
}



bool p300_getopts(Options* opt, int narg, char** varg) {

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

    opt->Set("xml-tasks",  xml_tasks);
    opt->Set("xml-events", xml_events);
    opt->Set("win-w",      std::to_string(P300_WIN_WIDTH));
    opt->Set("win-h",      std::to_string(P300_WIN_HEIGHT));

	return result;
};




#endif
