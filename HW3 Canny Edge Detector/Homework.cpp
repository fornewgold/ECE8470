/*
ECE 847
Assignment #3

The function of this code is to do Canny edge detection and match the template

usage: usage: homework sigma image [template]

Xi Zhao
10/4/2013
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

const double Pi = 4 * atan(1);

const double f = 2.5;
const double p = 5;
const double ratio = 5;

// Redirect file path
char * MyRedirectPath(const char * filename)
{
	string path ("../../images/");
	string path_filename = path + filename;
	char * path_filename_c = new char [path_filename.length()+1];

	strcpy (path_filename_c, path_filename.c_str());

	return path_filename_c;
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
	for (int y = 0 ; y < img_x.Height() ; y++)
		for (int x = 0 ; x < img_x.Width() ; x++)
			(* out)(x, y) = sqrt(img_x(x, y) * img_x(x, y) + img_y(x, y) * img_y(x, y));
}

// Compute phase of image
void MyGetPhase(const ImgFloat & img_x, const ImgFloat & img_y, ImgFloat * out)
{
	for (int y = 0 ; y < img_x.Height() ; y++)
		for (int x = 0 ; x < img_x.Width() ; x++)
			(* out)(x, y) = atan2f(img_y(x, y), img_x(x, y));
}

void MyGetDirection(float phase, int* dx, int* dy)
{
	if (phase < 0)  phase += (float) blepo_ex::Pi;

	double pi8 = blepo_ex::Pi / 8;

	if      (phase <     pi8)  { *dx = 1;  *dy =  0; }
	else if (phase < 3 * pi8)  { *dx = 1;  *dy =  1; }
	else if (phase < 5 * pi8)  { *dx = 0;  *dy =  1; }
	else if (phase < 7 * pi8)  { *dx = 1;  *dy = -1; }
	else                       { *dx = 1;  *dy =  0; }
}

void MyNonMaximumSuppression(const ImgFloat& magnitude, const ImgFloat& phase, ImgFloat* out)
{
	out->Reset(magnitude.Width(), magnitude.Height());
	Set(out, 0);

	int dx, dy;

	for (int y = 1 ; y < magnitude.Height()-1 ; y++)
	{
		for (int x = 1 ; x < magnitude.Width()-1 ; x++)
		{
			MyGetDirection( phase(x, y), &dx, &dy );
			float val0 = magnitude(x, y);
			float val1 = magnitude(x+dx, y+dy);
			float val2 = magnitude(x-dx, y-dy);
			(*out)(x, y) = (val0 >= val1 && val0 >= val2) ? val0 : 0;
		}
	}
}

// Compute thresholds
void MyGetThresholds(const ImgFloat & in, float & high, float & low)
{
	float * histogram = NULL;
	histogram = new float[in.Height() * in.Width()];

	for (int y = 0 ; y < in.Height() ; y++)
		for (int x = 0 ; x < in.Width() ; x++) 
			histogram[x + y * in.Width()] = in(x, y);

	sort(histogram, histogram + in.Height() * in.Width());

	high = histogram[(int) (in.Height() * in.Width() * (1 - p / 100))];
	low = high / ratio;

	delete histogram;
}

// Double threshold image with high and low thresholded image
void MyDoubleThreshold(const ImgBinary & img_high, const ImgBinary & img_low, ImgBinary * img_double)
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

// Canny edge detection
void MyCannyEdge(const ImgBgr img_1, ImgBinary * out, double sigma, bool print)
{
	out->Reset(img_1.Width(), img_1.Height());
	Set(out, 0);

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

	// Convert input image format
	ImgFloat img_2;

	Convert(img_1, &img_2);

	// Compute image gradient and non-maximum suppression
	ImgFloat img_x(IMG_WIDTH, IMG_HEIGHT);
	ImgFloat img_y(IMG_WIDTH, IMG_HEIGHT);
	Set(&img_x, 0);
	Set(&img_y, 0);

	MySeparableConvolve(img_2, gdk, gk, w_GDK, &img_x);
	MySeparableConvolve(img_2, gk, gdk, w_GK, &img_y);

	delete gk, gdk;

	ImgFloat img_magnitude(IMG_WIDTH, IMG_HEIGHT);
	ImgFloat img_phase(IMG_WIDTH, IMG_HEIGHT);
	ImgFloat img_magnitude_suppression(IMG_WIDTH, IMG_HEIGHT);
	Set(&img_magnitude, 0);
	Set(&img_phase, 0);
	Set(&img_magnitude_suppression, 0);

	MyGetMagnitude(img_x, img_y, &img_magnitude);
	MyGetPhase(img_x, img_y, &img_phase);
	MyNonMaximumSuppression(img_magnitude, img_phase, &img_magnitude_suppression);

	// Compute thresholds
	float threshold_high = 0;
	float threshold_low = 0;

	MyGetThresholds(img_magnitude_suppression, threshold_high, threshold_low);

	//// Threshold image
	ImgBinary img_high, img_low, img_canny;

	Threshold(img_magnitude_suppression, threshold_high, &img_high);
	Threshold(img_magnitude_suppression, threshold_low, &img_low);
	MyDoubleThreshold(img_high, img_low, out);

	if (print)
	{
		Figure fig_x = Figure("X", IMG_WIDTH * 1, IMG_HEIGHT * 0, true);
		Figure fig_y = Figure("Y", IMG_WIDTH * 2, IMG_HEIGHT * 0, true);
		Figure fig_magnitude = Figure("Magnitude", IMG_WIDTH * 0, IMG_HEIGHT * 1, true);
		Figure fig_phase = Figure("Phase", IMG_WIDTH * 1, IMG_HEIGHT * 1, true);
		Figure fig_suppressed = Figure("Suppressed", IMG_WIDTH * 2, IMG_HEIGHT * 1, true);

		fig_x.Draw(img_x);
		fig_y.Draw(img_y);
		fig_magnitude.Draw(img_magnitude);
		fig_phase.Draw(img_phase);
		fig_suppressed.Draw(img_magnitude_suppression);
	}
}

void MyChamfer(const ImgGray& img, ImgInt* chamfer_dist)
{
	chamfer_dist->Reset(img.Width(), img.Height());

	int infinity = img.Width() * img.Height() + 1;
	int x, y;

	// 1st pass
	for (y=0 ; y<img.Height() ; y++)
	{
		for (x=0 ; x<img.Width() ; x++)
		{
			if (img(x, y))  (*chamfer_dist)(x, y) = 0;
			else
			{
				int dist = infinity;
				if (y > 0)  dist = blepo_ex::Min( dist, (*chamfer_dist)(x, y-1) + 1);
				if (x > 0)  dist = blepo_ex::Min( dist, (*chamfer_dist)(x-1, y) + 1);
				(*chamfer_dist)(x, y) = dist;
			}
		}
	}

	// 2nd pass
	for (y=img.Height()-1 ; y>=0 ; y--)
	{
		for (x=img.Width()-1 ; x>=0 ; x--)
		{
			if (img(x, y))  (*chamfer_dist)(x, y) = 0;
			else
			{
				int dist = (*chamfer_dist)(x, y);
				if (y < img.Height()-1)  dist = blepo_ex::Min( dist, (*chamfer_dist)(x, y+1) + 1);
				if (x < img.Width ()-1)  dist = blepo_ex::Min( dist, (*chamfer_dist)(x+1, y) + 1);
				(*chamfer_dist)(x, y) = dist;
			}
		}
	}
}

// Match two edge images
void MyMatchEdge(const ImgBinary & img_1, const ImgBinary & img_2, ImgInt * out, int & x_p, int & y_p)
{
	int x_mapped, y_mapped;
	int val;
	int sum;
	int valid;
	int low = 0;
	int step = 0;
	float step_percent;

	int img_1_size = img_1.Width() * img_1.Height();
	int img_2_size = 0;
	for (int y = 0 ; y < img_2.Height() ; y++)
		for (int x = 0 ; x < img_2.Width() ; x++)
			if (img_2(x, y) != 0)
				img_2_size++;

	// Initialize output image
	out->Reset(img_1.Width(), img_1.Height());
	Set(out, 0);

	// Computer Chamfer distance
	ImgGray img_1_Gray;
	ImgInt img_1_dist;

	Convert(img_1, &img_1_Gray);
	MyChamfer(img_1_Gray, &img_1_dist);

	// Match edges
	cout << endl;
	cout << "matching edges..." << endl;

	for (int y_1 = 0 ; y_1 < img_1.Height() ; y_1++)
	{
		for (int x_1 = 0 ; x_1 < img_1.Width() ; x_1++) 
		{
			step_percent = 100 * step / img_1_size;
			cout << step_percent << "%";

			sum = 0;
			valid = 0;

			for (int y_2 = 0 ; y_2 < img_2.Height() ; y_2++)
			{
				for (int x_2 = 0 ; x_2 < img_2.Width() ; x_2++) 
				{
					val = 0;

					x_mapped = x_1 + x_2 - img_2.Width() / 2;
					y_mapped = y_1 + y_2 - img_2.Height() / 2;

					if ((x_mapped >= 0) && (x_mapped < img_1.Width()) && (y_mapped >= 0) && (y_mapped < img_1.Height()))
					{
						if (img_2(x_2, y_2) != 0)
						{
							val = img_1_dist(x_mapped, y_mapped);
							valid++;
						}

						sum += val;
					}
				}
			}

			sum *= img_2_size / valid;

			if ((x_1 >= 0 + img_2.Width() / 2 ) && (x_1 < img_1.Width() - img_2.Width() / 2) && (y_1 >= 0 + img_2.Height() / 2 ) && (y_1 < img_1.Height() - img_2.Height() / 2))
			{
				if ((x_1 == 0 + img_2.Width() / 2) && (y_1 == 0 + img_2.Height() / 2))
				{
					low = sum;
					x_p = x_1;
					y_p = y_1;
				}
				else if (sum < low)
				{
					low = sum;
					x_p = x_1;
					y_p = y_1;
				}
			}
			low = (sum < low) ? sum : low;

			(* out)(x_1, y_1) = sum;

			step++;

			cout << "\r";
		}
	}

	cout << "\r100%" << endl;
}

int main(int argc, const char* argv[], const char* envp[])
{
	// Check arguments
	if (!((argc == 3) || (argc == 4)))
	{
		cerr << "number of arguments must be 3 or 4!" << endl;
		cerr << "usage: homework sigma image [template]" << endl;
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
		// Read sigma
		double sigma;

		if (!(sigma = atof(argv[1]))) 
		{
			cerr << "usage: homework sigma image [template]" << endl;
			system("pause");
			exit(1);
		}

		// Load image
		ImgBgr img_1; 

		Load(MyRedirectPath(argv[2]), &img_1);
		if (((IMG_WIDTH == 0)||(IMG_HEIGHT == 0))) 
		{
			cerr << "cannot open zero size image!" <<  endl;
			system("pause");
			exit(1);
		}

		// Detect Canny Edge
		ImgBinary img_canny;
		MyCannyEdge(img_1, &img_canny, sigma, true);

		// Computer Chamfer distance
		ImgGray img_1_Gray;
		ImgInt img_1_dist;

		Convert(img_canny, &img_1_Gray);
		MyChamfer(img_1_Gray, &img_1_dist);

		// Match template
		ImgBgr img_template; 
		ImgBinary img_template_canny;
		ImgInt img_inverse_probability;
		ImgBgr img_matching = img_1;

		Figure fig_input = Figure("Input Image", 0, 0, true);
		Figure fig_canny = Figure("Canny", IMG_WIDTH * 0, IMG_HEIGHT * 2, true);
		Figure fig_dist = Figure("Chamfer Distance", IMG_WIDTH * 1, IMG_HEIGHT * 2, true);

		fig_input.Draw(img_1);
		fig_canny.Draw(img_canny);
		fig_dist.Draw(img_1_dist);

		if (argc == 4)
		{
			// Load image
			Load(MyRedirectPath(argv[3]), &img_template);
			if (((IMG_WIDTH == 0)||(IMG_HEIGHT == 0))) 
			{
				cerr << "cannot open zero size image!" <<  endl;
				system("pause");
				exit(1);
			}

			// Detect Canny edge
			MyCannyEdge(img_template, &img_template_canny, sigma, false);

			// Match Edge
			int x_p = 0;
			int y_p = 0;

			MyMatchEdge(img_canny, img_template_canny, &img_inverse_probability, x_p, y_p);
			cout << endl << "peak probabilty at: ("<< x_p << "," << y_p << ")"<< endl;

			// Draw rectangle
			Rect loc(x_p - img_template.Width() / 2, y_p - img_template.Height() / 2, x_p + img_template.Width() / 2, y_p + img_template.Height() / 2);
			Bgr rect_color = rect_color.GREEN;
			DrawRect(loc, &img_matching, rect_color.GREEN, 1);

			Figure fig_template = Figure("Template", IMG_WIDTH * 0, IMG_HEIGHT * 3, true);
			Figure fig_inverse_probability = Figure("Probability", IMG_WIDTH * 2, IMG_HEIGHT * 2, true);
			Figure fig_matching = Figure("Matching", IMG_WIDTH * 3, IMG_HEIGHT * 2, true);

			fig_template.Draw(img_template_canny);
			fig_inverse_probability.Draw(img_inverse_probability);
			fig_matching.Draw(img_matching);
		}

		// Display image in figure window. 

		// Loop forever until user presses Ctrl-C in terminal window.
		EventLoop();
	}
	catch (const Exception& e) 
	{
		e.Display();    // display exception to user in a popup window 
	}

	return 0;
}
