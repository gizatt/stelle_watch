#include <math.h>
// Modified from https://github.com/Inseckto/HSV-to-RGB/blob/master/HSV2RGB.c

void hsv2rgb(float h, float s, float v, uint8_t * rgb) {
	float r, g, b;
	int i = floor(h * 6);
	float f = h * 6 - i;
	float p = v * (1 - s);
	float q = v * (1 - f * s);
	float t = v * (1 - (1 - f) * s);
	
	switch (i % 6) {
		case 0: r = v, g = t, b = p; break;
		case 1: r = q, g = v, b = p; break;
		case 2: r = p, g = v, b = t; break;
		case 3: r = p, g = q, b = v; break;
		case 4: r = t, g = p, b = v; break;
		case 5: r = v, g = p, b = q; break;
	}
	
    rgb[0] = r*255;
    rgb[1] = g*255;
    rgb[2] = b*255;
}