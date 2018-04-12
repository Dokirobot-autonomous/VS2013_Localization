#pragma once

#include <opencv2\highgui\highgui.hpp>

struct MouseEventParam
{
	int x;
	int y;
	int event;
	int flags;
};

class MouseEventImage
{
private:
	MouseEventParam params;
	std::string window_name;
	static void callBackFunc(int eventType, int x, int y, int flags, void* userdata)
	{
		MouseEventParam *ptr = static_cast<MouseEventParam*> (userdata);

		ptr->x = x;
		ptr->y = y;
		ptr->event = eventType;
		ptr->flags = flags;
	}

public:

	MouseEventImage()
	{
	}

	~MouseEventImage()
	{
	}

	void setMouseEvent(std::string window_name){
		this->window_name = window_name;
		//cv::imshow(window_name, img);
		cv::setMouseCallback(window_name, callBackFunc, &params);
	}

	void waitLeftButton(){
		while (true){
			cv::waitKey(20);
			if (params.event == cv::EVENT_LBUTTONDOWN){
				return;
			}
		}
	}

	void resetEvent(){ 
		params.x = params.y = 0;
		params.event = cv::EVENT_MOUSEMOVE;
	}

	int getX() const {
		return params.x;
	}
	int getY() const {
		return params.y;
	}
	int getEvent() const {
		return params.event;
	}
	int getFlags() const {
		return params.flags;
	}

};

