#include <stdlib.h>
#include <iostream>

#include "set_color.h" 

#ifndef MAX
#define MAX(A, B) (A > B ? A : B)
#endif

#ifndef MIN
#define MIN(A, B) (A < B ? A : B)
#endif

#ifndef CLAMP
#define CLAMP(X, Xmin, Xmax) (X < Xmin ? Xmin : ( X > Xmax ? Xmax : X ) )
#endif 



RGB_ hsv2rgb(const HSV &_hsv)
{
	float h = (int)(_hsv.h/60);
	float f = _hsv.h/60 - h;
	float s = _hsv.s;
	float v = _hsv.v;	
	float p = v * (1 - s);
	float q = v * (1 - f * s);
	float t = v * (1 - (1 -f) * s);

	// std::cout << "h : " << h << std::endl;
	// std::cout << "f : " << f << std::endl;
	// std::cout << "s : " << s << std::endl;
	// std::cout << "v : " << v << std::endl;
	// std::cout << "p : " << p << std::endl;
	// std::cout << "q : " << q << std::endl;
	// std::cout << "t : " << t << std::endl;

	float r;
	float g;
	float b;

	if (h == 0) 
	{
		r = v;
		g = t;
		b = p;
	}
	else if (h == 1)
	{
		r = q;
		g = v;
		b = p;
	}
	else if (h == 2)
	{
		r = p;
		g = v;
		b = t;
	}
	else if (h == 3)
	{
		r = p;
		g = q;
		b = v;
	}
	else if (h == 4)
	{
		r = t;
		g = p;
		b = v;
	}
	else
	{
		r = v;
		g = p;
		b = q;
	}

	RGB_ rgb;
	rgb.r = CLAMP( r * 255.0f, 0, 255);
	rgb.g = CLAMP( g * 255.0f, 0, 255);
	rgb.b = CLAMP( b * 255.0f, 0, 255);	
	return rgb;
}

HSV rgb2hsv(const RGB_ &_rgb)
{
	float r = _rgb.r / 255.0f;
	float g = _rgb.g / 255.0f;
	float b = _rgb.b / 255.0f;

	float max = MAX(r, MAX(g, b));
	float min = MIN(r, MIN(g, b));

	float h;
	float s;
	float v; 

	if (max == min) h = 0;
	else if (max == r && g >= b) h = 60 * (g - b)/(max - min);
	else if (max == r && g < b) h = 60 * (g - b)/(max - min) + 360;
	else if (max == g) h = 60 * (b - r)/(max - min) + 120;
	else h = 60 * (r - g)/(max - min) + 240;

	if (max == 0) s = 0;
	else s = 1 - min/max;

	v = max;

	HSV hsv;
	hsv.h = CLAMP(h, 0, 359.99f);
	hsv.s = CLAMP(s, 0, 1);
	hsv.v = CLAMP(v, 0, 1);

	return hsv;
}

std::vector<RGB_> generateUniformColors(int _n, float start_degree, float end_degree)
{
	start_degree = CLAMP(start_degree, 0, 359.99f);
	end_degree = CLAMP(end_degree, 0, 359.99f);

	std::vector<RGB_> result_rgbs;

	for (int i = 0; i < _n; ++i)
	{
		float h = start_degree + ( end_degree - start_degree ) / (_n - 1 ) * i;
		float s = 0.5f;
		float v = 0.7f;

		HSV hsv;
		hsv.h = h;
		hsv.s = s;
		hsv.v = v;

		RGB_ rgb = hsv2rgb(hsv);
		result_rgbs.push_back(rgb);
	}

	return result_rgbs;
}
