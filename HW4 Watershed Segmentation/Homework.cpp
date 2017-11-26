/*
ECE 847
Assignment #4

This code is to implement the simplified Vincent-Soille marker-based watershed segmentation algorithm.    

usage: homework filename threshold

Xi Zhao
11/6/2013
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

const int threshold_1 = 70;
const int threshold_2 = 30;
const float sigma = 3;

const double Pi = 4 * atan(1);

const double f = 2.5;

// Redirect file path
char * MyRedirectPath(const char * filename)
{
	string path ("../../images/");
	string path_filename = path + filename;
	char * path_filename_c = new char [path_filename.length()+1];

	strcpy (path_filename_c, path_filename.c_str());

	return path_filename_c;
}

// Inverse binary Image
void MyInverseBinaryImage(ImgBinary * img)
{
	blepo::ImgBinary::Iterator p;

	for (p = img->Begin() ; p != img->End() ; p++) 
	{
		*p = !(*p);
	}
}

// Detect edges
void MyDetectEdge(const ImgInt & in, ImgBinary * out)
{
	out->Reset(in.Width(), in.Height());
	Set(out, 0);

	int neighbor[4];

	for (int y = 0 ; y < in.Height() ; y++)
	{
		for (int x = 0 ; x < in.Width() ; x++)
		{
			neighbor[0] = (y == 0) ? in(x, y) : in(x, y - 1);
			neighbor[1] = (x == (in.Width() - 1)) ? in(x, y) : in(x + 1, y);
			neighbor[2] = (y == (in.Height() - 1)) ? in(x, y) : in(x, y + 1);
			neighbor[3] = (x == 0) ? in(x, y) : in(x - 1, y);

			if ((in(x, y) != neighbor[0]) || (in(x, y) != neighbor[1]) || (in(x, y) != neighbor[2]) || (in(x, y) != neighbor[3]) )
				(*out)(x, y) = true;
		}
	}
}

// Get Chamfer distance
void MyChamferDistance(const ImgGray & img, ImgInt * distance)
{
	distance->Reset(img.Width(), img.Height());
	Set(distance, 0);

	int infinity = img.Width() * img.Height() + 1;
	int x, y;

	// 1st pass
	for (y = 0 ; y < img.Height() ; y++)
	{
		for (x = 0 ; x < img.Width() ; x++)
		{
			if (img(x, y))
				(*distance)(x, y) = 0;
			else
			{
				int dist = infinity;
				if (y > 0)  
					dist = blepo_ex::Min(dist, (*distance)(x, y-1) + 1);
				if (x > 0)  
					dist = blepo_ex::Min(dist, (*distance)(x-1, y) + 1);
				(*distance)(x, y) = dist;
			}
		}
	}

	// 2nd pass
	for (y = img.Height() - 1 ; y >= 0 ; y--)
	{
		for (x = img.Width() - 1 ; x >= 0 ; x--)
		{
			if (img(x, y))  
				(*distance)(x, y) = 0;
			else
			{
				int dist = (*distance)(x, y);
				if (y < img.Height() - 1)  
					dist = blepo_ex::Min(dist, (*distance)(x, y+1) + 1);
				if (x < img.Width () - 1)  
					dist = blepo_ex::Min(dist, (*distance)(x+1, y) + 1);
				(*distance)(x, y) = dist;
			}
		}
	}
}

// Get the label of a neighbor
int MyGetNeighborLabel(const Point & pt, const ImgInt & label)
{
	int w = label.Width() - 1;
	int h = label.Height() - 1;
	int temp;

	if (pt.x > 0) 
	{ 
		temp = label(pt.x-1, pt.y);  
		if (temp >= 0)  
			return temp; 
	}
	if (pt.y > 0) 
	{ 
		temp = label(pt.x, pt.y-1);  
		if (temp >= 0)  
			return temp; 
	}
	if (pt.x < w) 
	{ 
		temp = label(pt.x+1, pt.y);  
		if (temp >= 0)  
			return temp; 
	}
	if (pt.y < h) 
	{ 
		temp = label(pt.x, pt.y+1);  
		if (temp >= 0)  
			return temp; 
	}
	return -1;
}

// Expand frontier
void MyExpandFrontier(int x, int y, unsigned char gray_lvl, int label, const ImgGray & in, ImgInt * out, Array<Point> * frontier)
{
	if ( in(x, y) <= gray_lvl && (*out)(x, y) < 0)
	{
		(*out)(x, y) = label;
		frontier->Push(Point(x, y));
	}
}

// Expand boundary
template <typename T>
void MyExpandBoundary(const Image<T> & in, Image<T> * out, std::vector<Point> * frontier, const Point & p, typename Image<T>::Pixel color, typename Image<T>::Pixel fill_color)
{
	Image<T>::Pixel pixel  = in(p.x, p.y);
	Image<T>::PixelRef pixel_2 = (*out)(p.x, p.y);

	if (pixel == color && pixel_2 != fill_color)
	{
		frontier->push_back(p);
		pixel_2 = fill_color;
	}
}


// Floodfill image
template <typename T>
void My4Floodfill(const Image<T> & in, int x, int y, typename Image<T>::Pixel fill_color, Image<T> * out)
{
	out->Reset(in.Width(), in.Height()); 

	if (x < 0 || y < 0 || x >= in.Width() || y >= in.Height())
	{
		cerr << "coordinates are out of boundary!";
		exit(1);
	}

	std::vector<Point> frontier;
	Point p;

	if (fill_color == (*out)(x,y))  
		return;

	Image<T>::Pixel color = in(x, y);

	frontier.push_back(Point(x,y));

	(*out)(x, y) = fill_color;

	while (!frontier.empty())
	{
		p = frontier.back();
		frontier.pop_back();

		if (p.x > 0)  
			MyExpandBoundary(in, out, &frontier, Point(p.x - 1, p.y), color, fill_color);
		if (p.x < in.Width() - 1)  
			MyExpandBoundary(in, out, &frontier, Point(p.x + 1, p.y), color, fill_color);
		if (p.y > 0)  
			MyExpandBoundary(in, out, &frontier, Point(p.x, p.y - 1), color, fill_color);
		if (p.y < in.Height() - 1)  
			MyExpandBoundary(in, out, &frontier, Point(p.x, p.y + 1), color, fill_color);
	}
}

// Watershed gray image
void MyWatershed(const ImgGray & in, ImgInt * out, bool marker_based)
{
	// Initialization
	out->Reset(in.Width(), in.Height());
	Set(out, -1);

	const int n_gray = 256;
	int next_label = 0;

	ImgInt img_int;

	Convert(in, &img_int);

	// Make pixel list
	Array<Point> pixels[n_gray];
	{
		const unsigned char * p = in.Begin();
		for (int y = 0 ; y < in.Height() ; y++)
			for (int x = 0 ; x < in.Width() ; x++)
				pixels[*p++].Push(Point(x, y));
	}

	for (int gray = 0 ; gray < n_gray ; gray++)
	{
		Array<Point> frontier, frontier_2;

		// Grow catchment basin
		for (int i = 0 ; i < pixels[gray].Len() ; i++)
		{
			const Point & pt = pixels[gray][i];
			int lab = MyGetNeighborLabel(pt, *out);

			if (lab >= 0)
			{
				(*out)(pt.x, pt.y) = lab;
				frontier.Push(pt);
			}
		}

		// Expand frontier
		do
		{
			while (frontier.Len() > 0)
			{				
				Point pt = frontier.Pop();
				Point pt2;

				const int w = in.Width() - 1;
				const int h = in.Height() - 1;

				int lab = (*out)(pt.x, pt.y);

				if (pt.x > 0) 
					MyExpandFrontier(pt.x - 1, pt.y, gray, lab, in, out, &frontier_2);
				if (pt.x < w) 
					MyExpandFrontier(pt.x + 1, pt.y, gray, lab, in, out, &frontier_2);
				if (pt.y > 0) 
					MyExpandFrontier(pt.x, pt.y - 1, gray, lab, in, out, &frontier_2);
				if (pt.y < h) 
					MyExpandFrontier(pt.x, pt.y + 1, gray, lab, in, out, &frontier_2);

				if (pt.x > 0 && pt.y > 0) 
					MyExpandFrontier(pt.x - 1, pt.y - 1, gray, lab, in, out, &frontier_2);
				if (pt.x < w && pt.y < h) 
					MyExpandFrontier(pt.x + 1, pt.y + 1, gray, lab, in, out, &frontier_2);
				if (pt.x > 0 && pt.y < h) 
					MyExpandFrontier(pt.x - 1, pt.y + 1, gray, lab, in, out, &frontier_2);
				if (pt.x < w && pt.y > 0) 
					MyExpandFrontier(pt.x + 1, pt.y - 1, gray, lab, in, out, &frontier_2);
			}

			frontier = frontier_2;
			frontier_2.Reset();

		} while (frontier.Len() > 0);

		// Create new catchment basin
		if (!marker_based || gray == 0)
		{
			for (int i = 0 ; i < pixels[gray].Len() ; i++)
			{
				const Point & pt = pixels[gray][i];

				if ((*out)(pt.x, pt.y) < 0)  
					My4Floodfill(img_int, pt.x, pt.y, next_label++, out);
			}
		}
	}
}

// Threshold gray image
void MyThreshold(const ImgGray & in, unsigned char threshold, ImgBinary * out)
{
	out->Reset(in.Width(), in.Height());
	Set(out, 0);

	ImgGray::ConstIterator p;
	ImgBinary::Iterator q;

	for (p = in.Begin(), q = out->Begin() ; p != in.End() ; p++, q++)
	{
		*q = (*p >= threshold);
	}
}

// Get markers
void GetMarker(const ImgGray & in, const int threshold, ImgBinary * out)
{
	ImgBinary img_thresholded;
	ImgGray img_thresholded_gray;
	ImgInt img_chamfer;
	ImgGray img_chamfer_gray;
	ImgInt img_marker_label;
	ImgBinary img_marker_edge;
	ImgBinary img_marker;

	MyThreshold(in, threshold, &img_thresholded);
	MyInverseBinaryImage(&img_thresholded);

	Figure fig_thresholded = Figure("Thresholded Image", in.Width() * 1, in.Height() * 0, true);
	fig_thresholded.Draw(img_thresholded);

	Convert(img_thresholded, &img_thresholded_gray);
	MyChamferDistance(img_thresholded_gray, &img_chamfer);

	Figure fig_chamfer = Figure("Chamfer Distance", in.Width() * 2, in.Height() * 0, true);
	fig_chamfer.Draw(img_chamfer);

	Convert(img_chamfer, &img_chamfer_gray);
	MyWatershed(img_chamfer_gray, &img_marker_label, false);

	Figure fig_marker_label = Figure("Non-marker-based Watershed Image", in.Width() * 3, in.Height() * 0, true);
	fig_marker_label.Draw(img_marker_label);

	MyDetectEdge(img_marker_label, &img_marker_edge);

	Figure fig_marker_edge = Figure("Edge", in.Width() * 4, in.Height() * 0, true);
	fig_marker_edge.Draw(img_marker_edge);

	Or(img_thresholded, img_marker_edge, out);

	Figure fig_marker = Figure("Marker", in.Width() * 1, in.Height() * 1, true);
	fig_marker.Draw(*out);
}

// Round double to long
long MyRound(double in)
{
	long out = (long) (in + 0.5);
	return out;
}

// Convolve image with kernals separately in x and y directions
void MySeparableConvolve(const ImgFloat & in, const float * x_GK, const float * y_GK, const int w, ImgFloat * out)
{
	out->Reset(in.Width(), in.Height());
	Set(out, 0);

	ImgFloat temp(in.Width(), in.Height());
	Set(&temp, 0);

	int w_half = (w - 1) / 2;

	// Convolve vetically
	for (int x = 0 ; x < in.Width() ; x++)
	{
		for (int y = 0 ; y < in.Height() ; y++)
		{
			float y_val = 0;

			for (int i = 0 ; i < w ; i++)
			{
				int y_hat = y + w_half - i;

				if ((y_hat >= 0) && (y_hat <= (in.Height() - 1)))
					y_val += y_GK[i] * in(x, y_hat);
				else if (y_hat < 0)
					y_val += y_GK[i] * in(x, 0);
				else 
					y_val += y_GK[i] * in(x, in.Height() - 1);
			}

			temp(x, y) = y_val;
		}
	}

	// Convolve horizontally
	for (int y = 0 ; y < in.Height() ; y++)
	{
		for (int x = 0 ; x < in.Width() ; x++)
		{
			float x_val = 0;

			for (int i = 0 ; i < w ; i++)
			{
				int x_hat = x + w_half - i;

				if ((x_hat >= 0) && (x_hat <= (in.Width() - 1)))
					x_val += x_GK[i] * temp(x_hat, y);
				else if (x_hat < 0)
					x_val += x_GK[i] * temp(0, y);
				else
					x_val += x_GK[i] * temp(in.Width() - 1, y);
			}

			(* out)(x, y) = x_val;
		}
	}
}

// Compute magnitude of image
void MyGetMagnitude(const ImgFloat & img_x, const ImgFloat & img_y, ImgFloat * out)
{
	out->Reset(img_x.Width(), img_x.Height());
	Set(out, 0);

	for (int y = 0 ; y < img_x.Height() ; y++)
		for (int x = 0 ; x < img_x.Width() ; x++)
			(* out)(x, y) = sqrt(img_x(x, y) * img_x(x, y) + img_y(x, y) * img_y(x, y));
}

// Get gradient magnitude
void MyGetGradientMagnitude(const ImgGray & in, ImgFloat * out)
{
	out->Reset(in.Width(), in.Height());
	Set(out, 0);

	bool print = false;

	// Construct 1D Gaussian convolution kernel
	long mu_GK = MyRound(f * sigma - 0.5);
	long w_GK = 2 * mu_GK + 1;
	float * gk = NULL;
	gk = new float[w_GK];
	float sum_GK = 0;

	for (int i = 0 ; i < w_GK ; i++)
	{
		gk[i] = exp ( - (i - mu_GK) * (i - mu_GK) / (2 * sigma * sigma));
		sum_GK += gk[i];
	}

	if (print)
	{
		cout << "1D Gaussian convolution kernel with f = 2.5 and sigma = " << sigma << " is:" << endl;
		cout << "[";
		for (int i = 0 ; i < w_GK ; i++)
			cout << " " << gk[i];
		cout << " ]" << endl;
	}

	for (int i = 0 ; i < w_GK ; i++)
		gk[i] /= sum_GK;

	if (print)
	{
		cout << "Normalized kernel is:" << endl;
		cout << "[";
		for (int i = 0 ; i < w_GK ; i++)
			cout << " " << gk[i];
		cout << " ]" << endl;

		cout << endl;
	}

	// Construct 1D Gaussian derivative convolution kernel
	long mu_GDK = MyRound(f * sigma - 0.5);
	long w_GDK = 2 * mu_GDK + 1;
	float * gdk = NULL;
	gdk = new float[w_GDK];
	float sum_GDK = 0;

	for (int i = 0 ; i < w_GDK ; i++)
	{
		gdk[i] = - (i - mu_GDK ) * exp ( - (i - mu_GDK) * (i - mu_GDK) / (2 * sigma * sigma));
		sum_GDK -= gdk[i] * i;
	}

	if (print)
	{
		cout << "1D Gaussian derivative convolution kernel with f = 2.5 and sigma = " << sigma << " is:" << endl;
		cout << "[";
		for (int i = 0 ; i < w_GDK ; i++)
			cout << " " << gdk[i];
		cout << " ]" << endl;
	}

	for (int i = 0 ; i < w_GDK ; i++)
		gdk[i] /= sum_GDK;

	if (print)
	{
		cout << "Normalized kernel is:" << endl;
		cout << "[";
		for (int i = 0 ; i < w_GDK ; i++)
			cout << " " << gdk[i];
		cout << " ]" << endl;
	}

	// Convolve separatly
	ImgFloat img_2;
	ImgFloat img_x;
	ImgFloat img_y;

	Convert(in, &img_2);
	MySeparableConvolve(img_2, gdk, gk, w_GDK, &img_x);
	MySeparableConvolve(img_2, gk, gdk, w_GK, &img_y);

	delete gk, gdk;

	// Compute image gradient
	MyGetMagnitude(img_x, img_y, out);

	//Figure fig_magnitude_x = Figure("Gradient X", in.Width() * 3, in.Height() * 1, true);
	//fig_magnitude_x.Draw(img_x);

	//Figure fig_magnitude_y = Figure("Gradient Y", in.Width() * 3, in.Height() * 1, true);
	//fig_magnitude_y.Draw(img_y);

	Figure fig_magnitude = Figure("Smoothed Gradient Magnitude", in.Width() * 0, in.Height() * 1, true);
	fig_magnitude.Draw(*out);
}

void MyAddMaker(const ImgGray & in, const ImgBinary & marker, ImgGray * out)
{
	out->Reset(in.Width(), in.Height());
	Set(out, 0);

	blepo::ImgGray::ConstIterator p_in;
	blepo::ImgBinary::ConstIterator p_marker;
	blepo::ImgGray::Iterator p_out;

	p_in = in.Begin();
	p_marker = marker.Begin();
	p_out = out->Begin();

	while (p_out != out->End())
	{
		if ((*p_marker) != true)
			(*p_out) = (*p_in);

		p_in++;
		p_marker++;
		p_out++;
	}

	//Figure fig_marked_magnitude = Figure("Marked Magnitude", in.Width() * 1, in.Height() * 2, true);
	//fig_marked_magnitude.Draw(*out);
}

// Show final segmentation based on input image
void MyDrawSegmentation(const ImgBgr & in, const ImgBinary & edge, ImgBgr * out)
{
	out->Reset(in.Width(), in.Height());

	blepo::ImgBgr::ConstIterator p_in;
	blepo::ImgBinary::ConstIterator p_edge;
	blepo::ImgBgr::Iterator p_out;

	p_in = in.Begin();
	p_edge = edge.Begin();
	p_out = out->Begin();

	Bgr edge_color = edge_color.GREEN;

	while (p_out != out->End())
	{
		if ((*p_edge) != true)
			(*p_out) = (*p_in);
		else
			(*p_out) = edge_color.GREEN;

		p_in++;
		p_edge++;
		p_out++;
	}

	Figure fig_segmented = Figure("Final Segmented Image", in.Width() * 2, in.Height() * 1, true);
	fig_segmented.Draw(*out);
}

int main(int argc, const char* argv[], const char* envp[])
{
	// Print suggested threshold
	cout << "Suggested thresholds are: " << endl;
	cout << "holes.pgm: " << threshold_1 << endl;
	cout << "cells_small.pgm: " << threshold_2 << endl;
	cout << endl;

	// Check arguments
	if (argc != 3)
	{
		cerr << "number of arguments must be two!" << endl;
		cerr << "usage: homework filename threshold" << endl;
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

	try 
	{
		// Read threshold
		int threshold_in;

		if (!(threshold_in = atoi(argv[2])))
		{
			cerr << "usage: homework filename threshold" << endl;
			system("pause");
			exit(1);
		}

		// Load image
		ImgBgr img_0;
		ImgGray img_1;

		Load(MyRedirectPath(argv[1]), &img_0);
		if (((img_0.Width() == 0)||(img_0.Height() == 0))) 
		{
			cerr << "cannot open zero size image!" <<  endl;
			system("pause");
			exit(1);
		}

		Figure fig_input = Figure("Input Image", 0, 0, true);
		fig_input.Draw(img_0);

		// Detect marker
		ImgBinary img_marker;

		Convert(img_0, &img_1);
		GetMarker(img_1, threshold_in, &img_marker);

		// Get gradient magnitude
		ImgFloat img_magenitude;
		ImgGray img_magenitude_gray;
		ImgInt img_lable;
		ImgGray img_marked_magenitude;
		ImgBinary img_edge;

		MyGetGradientMagnitude(img_1, &img_magenitude);

		Convert(img_magenitude, &img_magenitude_gray, true);
		MyAddMaker(img_magenitude_gray, img_marker, &img_marked_magenitude);

		MyWatershed(img_marked_magenitude, &img_lable, true);

		MyDetectEdge(img_lable, &img_edge);

		//Draw on input image
		ImgBgr img_segmented;
		MyDrawSegmentation(img_0, img_edge, &img_segmented);

		// Display image in figure window.
		//Figure fig_segmentation = Figure("Labels", img_0.Width() * 2, img_0.Height() * 2, true);
		//fig_segmentation.Draw(img_lable);

		//Figure fig_edge = Figure("Segment Edges", img_0.Width() * 3, img_0.Height() * 2, true);
		//fig_edge.Draw(img_edge);

		// Loop forever until user presses Ctrl-C in terminal window.
		EventLoop();
	}
	catch (const Exception& e) 
	{
		e.Display();    // display exception to user in a popup window 
	}

	return 0;
}