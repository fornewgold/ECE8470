/*
ECE 847
Assignment #1

The function of this code is to read a original image, draw and save a grey image based on the original image, read a mask image and mask the original image with mask image.

usage: homework imagename_1 imagename_2 imagename_3

imagename_1 is the filename of the original image.
imagename_2 is the filename of the drawn image.
imagename_3 is the filename of the mask image.

Xi Zhao
9/1/2013
*/

// Homework.cpp : Defines the entry point for the console application.
//

#include <afxwin.h>  // necessary for MFC to work properly
#include "Homework.h"
#include "../../src/blepo.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace blepo;

// Redirect file path
char * RedirectPath(const char * filename)
{
	string path ("../../images/");
	string path_filename = path + filename;
	char * path_filename_c = new char [path_filename.length()+1];

	strcpy (path_filename_c, path_filename.c_str());

	return path_filename_c;
}

// Set a square at the center of the image
Rect SetCenterSquare(ImgGray * img, int square_length)
{
	square_length = 100;

	int square_top, square_bottom, square_left, square_right;

	square_top = (img->Height() < square_length) ? 0 : (img->Height() - square_length) / 2;
	square_bottom = (img->Height() < square_length) ? img->Height() : (img->Height() + square_length) / 2;
	square_left = (img->Width() < square_length) ? 0 : (img->Width() - square_length) / 2;
	square_right = (img->Width() < square_length) ? img->Width() : (img->Width() + square_length) / 2;

	Rect center_square(square_left, square_top, square_right, square_bottom);

	return center_square;
}

// Mask a Bgr image with a Grey image
void MaskBgrGrey(const ImgBgr & original, const ImgGray & mask, ImgBgr * output)
{
	if ((original.Height() == mask.Height())&&(original.Width() == mask.Width()))
	{
		Bgr val_black;
		val_black.g = 0;
		val_black.b = 0;
		val_black.r = 0;

		const Bgr * p_1 = original.Begin();
		const unsigned char * p_2 = mask.Begin();
		Bgr * p_3;

		for (p_3 = output->Begin() ; p_3 !=output->End() ; p_3++) 
		{
			if (*p_2 != 0) 
				*p_3 = *p_1;
			else
				*p_3 = val_black;

			p_1++;
			p_2++;
		}
	}
	else 
		cerr << "unequal images size!" << endl;
}

int main(int argc, const char* argv[], const char* envp[])
{
	// Check input
	if (argc != 4) 
	{
		cerr << "number of arguments must be three!" << endl;
		cerr << "usage: homework filename1 filename2 filename3" << endl;
		system("pause");
		exit(1);
	}

	// Check JPEG file
	string jpg_1 = (".jpg");
	string jpg_2 = (".jpeg");
	string jpg_3 = (".jpe");
	string extension_checker (argv[2]);
	boolean is_jpeg = false;

	if (extension_checker.compare(extension_checker.size()-4, 4, jpg_1) == 0)
		is_jpeg = true;
	if (extension_checker.compare(extension_checker.size()-5, 5, jpg_2) == 0)
		is_jpeg = true;
	if (extension_checker.compare(extension_checker.size()-4, 4, jpg_3) == 0)
		is_jpeg = true;

	if (is_jpeg)
	{
		cerr << "the second file cannot be a JPEG image!" << endl;
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
		// Load image 1
		ImgBgr img_1; 
		Load(RedirectPath(argv[1]), &img_1);
		if (((img_1.Width() == 0)||(img_1.Height() == 0))) 
		{
			cerr << "cannot open zero size image!" <<  endl;
			system("pause");
			exit(1);
		}

		// Draw image 2
		ImgGray img_2(img_1.Width(), img_1.Height());
		Set(&img_2, 0);

		Set(&img_2, SetCenterSquare(&img_2, 100), 255);

		// Save image 2
		Save(img_2, RedirectPath(argv[2]), "bmp");

		// Load image 3
		ImgGray img_3; 
		Load(RedirectPath(argv[3]), &img_3);

		// Draw image 4
		ImgBgr img_4(img_1.Width(), img_1.Height()); 

		MaskBgrGrey(img_1, img_3, &img_4);

		//if ((img_1.Height() == img_3.Height())&&(img_1.Width() == img_3.Width())) {
		//	for (int y = 0 ; y < img_4.Height() ; y++)
		//		for (int x = 0 ; x < img_4.Width() ; x++) {
		//			if (img_3(x, y) != 0) 
		//				img_4(x, y) = img_1(x, y);
		//			else 
		//				img_4(x, y)  = val_black;
		//		}
		//}
		//else 
		//	cerr << "unequal images size!" << endl;

		// Display image in figure window. 
		Figure fig_1, fig_2, fig_3, fig_4;
		fig_1.Draw(img_1);
		fig_2.Draw(img_2);
		fig_3.Draw(img_3);
		fig_4.Draw(img_4);

		// Loop forever until user presses Ctrl-C in terminal window.
		EventLoop();
	}
	catch (const Exception& e) 
	{
		e.Display();    // display exception to user in a popup window 
	}

	return 0;
}
