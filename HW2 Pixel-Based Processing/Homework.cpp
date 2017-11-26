/*
ECE 847
Assignment #2

The function of this code is to read a specified fruit image, threshold the image, detect fruits, classiy fruit types and stems and output region properties.

usage: homework imagename

imagename is the filename of the specified fruit image.

Xi Zhao
9/20/2013
*/

// Homework.cpp : Defines the entry point for the console application.
//

#include <afxwin.h>  // necessary for MFC to work properly
#include "Homework.h"
#include "../../src/blepo.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define IMG_WIDTH img_1.Width()
#define IMG_HEIGHT img_1.Height()

using namespace blepo;

const double Pi=4*atan(1);
const unsigned char threshold_high = 200;
const unsigned char threshold_low = 100;
const int stem_threshold = 8;

// Redirect file path
char * RedirectPath(const char * filename)
{
	string path ("../../images/");
	string path_filename = path + filename;
	char * path_filename_c = new char [path_filename.length()+1];

	strcpy (path_filename_c, path_filename.c_str());
	return path_filename_c;
}

// Double threshold image with high and low thresholded image
void double_threshold(const ImgBinary & img_high, const ImgBinary & img_low, ImgBinary * img_double)
{
	// Initialize output image
	img_double->Reset(img_high.Width(), img_high.Height());
	Set(img_double, 0);

	for (int y = 0 ; y < img_high.Height() ; y++)
	{
		for (int x = 0 ; x < img_high.Width() ; x++) 
		{
			if (img_high(x, y) == 1) 
				FloodFill4(img_low, x, y, 1, img_double);
		}
	}
}

// Detect stem
// Remove the pixels from which the distance to out-banana area are less than stem_threshold,
// and then add pixel from which the distance to in-banana area are less than stem_threshold
void detect_stem(const ImgGray & img_banana_temp, ImgBgr & img_classified)
{
	ImgInt img_banana_temp2;
	ImgBinary img_banana_temp3;
	ImgGray img_banana_temp4;
	ImgInt img_banana_temp5;
	ImgBinary img_banana_temp6;
	ImgBinary img_banana_temp7;
	ImgBinary img_stem_temp;
	ImgBinary img_stem_temp2;
	ImgBinary img_stem_NR;
	ImgBinary img_stem_boundary_temp;

	Bgr stem_color;

	Chamfer(img_banana_temp, &img_banana_temp2);
	Threshold(img_banana_temp2, stem_threshold, &img_banana_temp3);

	Convert(img_banana_temp3, &img_banana_temp4);
	Chamfer(img_banana_temp4, &img_banana_temp5);
	Threshold(img_banana_temp5, stem_threshold, &img_banana_temp6);

	Convert(img_banana_temp, &img_banana_temp7);
	Xor(img_banana_temp7, img_banana_temp6, &img_stem_temp);

	// Remove noise from stem binary image, closing
	Erode3x3(img_stem_temp, &img_stem_temp2);
	Dilate3x3(img_stem_temp2, &img_stem_NR);

	// Get current stem boundary image			
	Erode3x3(img_stem_NR, &img_stem_boundary_temp);
	Xor(img_stem_NR, img_stem_boundary_temp, &img_stem_boundary_temp);

	// Draw stem boundary on final image with color
	for (int y = 0 ; y < img_banana_temp.Height(); y++)
		for (int x = 0 ; x < img_banana_temp.Width(); x++)
			if (img_stem_boundary_temp(x, y) == 1) 
				img_classified(x, y) = stem_color.MAGENTA;
}

int main(int argc, const char* argv[], const char* envp[])
{
	// Check arguments
	if (argc != 2) 
	{
		cerr << "number of arguments must be one!" << endl;
		cerr << "usage: homework filename" << endl;
		system("pause");
		exit(1);
	}

	// Initialize MFC and return if failure
	HMODULE hModule = ::GetModuleHandle(NULL);
	if (hModule == NULL || !AfxWinInit(hModule, NULL, ::GetCommandLine(), 0))
	{
		printf("Fatal Error: MFC initialization failed (hModule = %x)\n", hModule);
		return 1;
	}

	try {
		// Load image
		ImgBgr img_1; 

		Load(RedirectPath(argv[1]), &img_1);
		if (((IMG_WIDTH == 0)||(IMG_HEIGHT == 0))) 
		{
			cerr << "cannot open zero size image!" <<  endl;
			system("pause");
			exit(1);
		}

		// Convert input image format
		ImgGray img_2;

		Convert(img_1, &img_2);

		// Threshold image
		ImgBinary img_high, img_low, img_double, img_double_temp, img_double_NR;

		Threshold(img_2, threshold_high, &img_high);
		Threshold(img_2, threshold_low, &img_low);
		double_threshold(img_high, img_low, &img_double);

		// Remove noise from double thresholded binary image, opening
		Dilate3x3(img_double, &img_double_temp);
		Erode3x3(img_double_temp, &img_double_NR);

		// Detect region
		ImgInt img_lable;
		vector<ConnectedComponentProperties<ImgBinary::Pixel>> region_property;

		ConnectedComponents4(img_double_NR, &img_lable, &(region_property));

		// Get properties of regions
		ImgInt img_region_boundary(IMG_WIDTH, IMG_HEIGHT);
		Set(&img_region_boundary, 0);

		ImgBgr img_classified;
		img_classified = img_1;

		// For each region, output properties and classify region
		for (int i = 1; i < region_property.size(); i++)
		{
			// Create a isolated binary image for current region
			ImgBinary img_region_temp(IMG_WIDTH, IMG_HEIGHT);
			Set(&img_region_temp, false);

			for (int y = 0 ; y < IMG_HEIGHT ; y++)
			{
				for (int x = 0 ; x < IMG_WIDTH ; x++) 
				{
					if (img_lable(x, y) == i) 
						img_region_temp(x, y) = true;
				}
			}

			// Get properties of current region
			RegionProperties rp;
			RegionProps(img_region_temp, &rp);

			// Get current region boundary image
			ImgBinary img_region_boundary_temp;

			Erode3x3(img_region_temp, &img_region_boundary_temp);
			Xor(img_region_temp, img_region_boundary_temp, &img_region_boundary_temp);

			// Get perimeter of current region
			int perimeter = 0;

			for (int y = 0 ; y < IMG_HEIGHT ; y++)
			{
				for (int x = 0 ; x < IMG_WIDTH ; x++)
				{
					if (img_region_boundary_temp(x, y) == 1) 
					{
						img_region_boundary(x, y) = i;
						perimeter++;
					}
				}
			}

			// Get compactness of current region
			double compactness = 4 * Pi * rp.area / perimeter / perimeter;

			// classify current region
			int fruit_class = 0;

			if ((rp.eccentricity > 0.7) && (compactness < 0.4))
			{
				if (rp.area > 100)
					fruit_class = 1; // banana
			}
			else if ((rp.eccentricity < 0.7) && (compactness > 0.4))
			{
				if (rp.area > 5000)
					fruit_class = 2; // grapefruit
				else if (rp.area > 100)
					fruit_class = 3; // apple
			}

			// Set color for each type
			Bgr class_color, default_color;

			switch(fruit_class)
			{
			case 1:
				class_color = default_color.YELLOW;
				break;
			case 2:
				class_color = default_color.GREEN;
				break;
			case 3:
				class_color = default_color.RED;
				break;
			default:
				class_color = default_color.WHITE;
			}

			// draw the boundary of current region on final image with specified color
			for (int y = 0 ; y < IMG_HEIGHT ; y++)
				for (int x = 0 ; x < IMG_WIDTH ; x++)
					if (img_region_boundary_temp(x, y) == 1) 
						img_classified(x, y) = class_color;

			// draw the cross of current region on final image with specified color
			Point pt1, pt2, pt3, pt4;
			pt1.SetPoint(rp.xc - rp.major_axis_x, rp.yc - rp.major_axis_y);
			pt2.SetPoint(rp.xc + rp.major_axis_x, rp.yc + rp.major_axis_y);
			pt3.SetPoint(rp.xc - rp.minor_axis_x, rp.yc - rp.minor_axis_y);
			pt4.SetPoint(rp.xc + rp.minor_axis_x, rp.yc + rp.minor_axis_y);

			DrawLine(pt1, pt2, &img_classified, class_color, 1);
			DrawLine(pt3, pt4, &img_classified, class_color, 1);

			// Print properties of current region
			cout << endl;
			cout << "Regaion " << i << " properties:" << endl;
			cout << "m00=" << rp.m00 << endl;
			cout << "m10=" << rp.m10 << ", m01=" << rp.m01 << ", m11=" << rp.m11 << endl;
			cout << "m20=" << rp.m20 << ", m02=" << rp.m02 << endl;
			cout << "mu00=" << rp.mu00 << endl;
			cout << "mu10=" << rp.mu10 << ", mu01=" << rp.mu01 << ", mu11=" << rp.mu11 << endl;
			cout << "mu20=" << rp.mu20 << ", mu02=" << rp.mu02 << endl;
			cout << "compactness=" << compactness << endl; 
			cout << "eccentricity=" << rp.eccentricity << endl; 
			cout << "direction=" << rp.direction << endl;

			// detect banana stem if current region is classified as banana
			if (fruit_class == 1)
			{
				ImgGray img_banana_temp(IMG_WIDTH, IMG_HEIGHT);


				Set(&img_banana_temp, 255);

				// Create specific binary image for current banana
				for (int y = 0 ; y < IMG_HEIGHT ; y++)
				{
					for (int x = 0 ; x < IMG_WIDTH ; x++) 
					{
						if (img_lable(x, y) == i) 
							img_banana_temp(x, y) = 0;
					}
				}

				detect_stem(img_banana_temp, img_classified);
			}
		}

		// Display image in figure window. 
		Figure fig_1 = Figure("Input Image", 0, 0, true);
		Figure fig_2 = Figure("High-Thresholded Image", IMG_WIDTH, 0, true);
		Figure fig_3 = Figure("Low-Thresholded Image", IMG_WIDTH * 2, 0, true);
		Figure fig_4 = Figure("Double-Thresholded Image", 0, IMG_HEIGHT, true);
		Figure fig_5 = Figure("Noise-removed & Double-Thresholded Image", IMG_WIDTH, IMG_HEIGHT, true);
		Figure fig_6 = Figure("Labled Image", IMG_WIDTH * 2, IMG_HEIGHT, true);
		Figure fig_7 = Figure("Region Boundary", 0, IMG_HEIGHT * 2, true);
		Figure fig_8 = Figure("Classified Image", IMG_WIDTH, IMG_HEIGHT * 2, true);

		fig_1.Draw(img_1);
		fig_2.Draw(img_high);
		fig_3.Draw(img_low);
		fig_4.Draw(img_double);
		fig_5.Draw(img_double_NR);
		fig_6.Draw(img_lable);
		fig_7.Draw(img_region_boundary);
		fig_8.Draw(img_classified);

		// Loop forever until user presses Ctrl-C in terminal window.
		EventLoop();
	}
	catch (const Exception& e) 
	{
		e.Display();    // display exception to user in a popup window 
	}

	return 0;
}
