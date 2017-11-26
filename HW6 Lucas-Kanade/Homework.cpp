/*
ECE 847
Assignment #6

This code is to implement the Lucas-Kanade algorithm to detect and track features through a video sequence of images.   

usage: homework filename-format first-frame last-frame sigma window-size
Xi Zhao
12/7/2013
*/

// Homework.cpp : Defines the entry point for the console application.
//

#include <afxwin.h>  // necessary for MFC to work properly
#include <math.h>

#include "Homework.h"
#include "../../src/blepo.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace blepo;

// Redirect file path
char * MyRedirectPath(const char* filename)
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
void MySeparableConvolve(const ImgFloat& in, const float* x_GK, const float* y_GK, const int w, ImgFloat* out)
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

// Get directional gradient
void MyGetGradient(const ImgFloat& in, const float sigma, ImgFloat* img_x, ImgFloat* img_y, ImgFloat* img_xx, ImgFloat* img_xy, ImgFloat* img_yy)
{
	// Initialize
	img_x->Reset(in.Width(), in.Height());
	Set(img_x, 0);

	img_y->Reset(in.Width(), in.Height());
	Set(img_y, 0);

	img_xx->Reset(in.Width(), in.Height());
	Set(img_xx, 0);

	img_xy->Reset(in.Width(), in.Height());
	Set(img_xy, 0);

	img_yy->Reset(in.Width(), in.Height());
	Set(img_yy, 0);

	const bool print = false;
	const float f = 2.5;

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

	// Convolve separatly, compute image gradient
	MySeparableConvolve(in, gdk, gk, w_GDK, img_x);
	MySeparableConvolve(in, gk, gdk, w_GK, img_y);

	delete gk, gdk;

	// Compute image secondary gradient
	Multiply(*img_x, *img_x, img_xx);
	Multiply(*img_x, *img_y, img_xy);
	Multiply(*img_y, *img_y, img_yy);

	//Figure fig_x = Figure("Gradient X", in.Width() * 0, in.Height() * 1, true);
	//fig_x.Draw(*img_x);

	//Figure fig_y = Figure("Gradient Y", in.Width() * 0, in.Height() * 1, true);
	//fig_y.Draw(*img_y);

	//Figure fig_xx = Figure("Gradient XX", in.Width() * 1, in.Height() * 1, true);
	//fig_xx.Draw(*img_xx);

	//Figure fig_xy = Figure("Gradient XY", in.Width() * 1, in.Height() * 1, true);
	//fig_xy.Draw(*img_xy);

	//Figure fig_yy = Figure("Gradient YY", in.Width() * 1, in.Height() * 1, true);
	//fig_yy.Draw(*img_yy);
}



// Calculate sum of the values in the window
void MySumGradient(int size, ImgFloat* gradxx, ImgFloat* gradxy, ImgFloat* gradyy)
{
	ImgFloat tmp;
	ImgFloat kernel_horiz(size,1), kernel_vert(1,size);
	Set(&kernel_horiz, 1);
	Set(&kernel_vert, 1);

	Convolve(*gradxx, kernel_horiz, &tmp);
	Convolve(tmp, kernel_vert, gradxx);
	Convolve(*gradxy, kernel_horiz, &tmp);
	Convolve(tmp, kernel_vert, gradxy);
	Convolve(*gradyy, kernel_horiz, &tmp);
	Convolve(tmp, kernel_vert, gradyy);
}

// Compare values of features
bool MyCompareFeatureValue(const FeaturePoint& feature_1, const FeaturePoint& feature_2)
{
	return feature_1.val > feature_2.val;
}

// Detect Tomasi-Kanade cornerness
void MyDetectFeature(const ImgGray in, vector<FeaturePoint>* selected_features)
{
	const int win_size = 3;
	const int w = (win_size - 1) / 2;
	const int min_eigenvalue = 150;
	const float sigma = 2;
	float min_eig;

	vector<FeaturePoint> features;

	ImgFloat img_x, img_y, img_xx, img_xy, img_yy;

	// Compute gradients
	ImgFloat in_f;
	Convert(in, &in_f);

	MyGetGradient(in_f, sigma, &img_x, &img_y, &img_xx, &img_xy, &img_yy);
	MySumGradient(win_size, &img_xx, &img_xy, &img_yy);	

	// Find features
	for (int y = w ; y < in.Height() - w ; y++)
	{
		for (int x = w ; x < in.Width() - w ; x++)
		{
			float gxx = img_xx(x, y);
			float gxy = img_xy(x, y);
			float gyy = img_yy(x, y);

			min_eig = (float) ((gxx + gyy - sqrt((gxx - gyy) * (gxx - gyy) + 4 * gxy * gxy)) / 2.0);

			if (min_eig > min_eigenvalue)
				features.push_back(FeaturePoint((float)x, (float)y, min_eig));
		}
	}

	// Non-maximal suppression
	sort(features.begin(), features.end(), MyCompareFeatureValue);

	for (vector<FeaturePoint>::iterator it_1 = features.begin() ; it_1 != features.end() ; ++it_1)
	{
		bool is_max = true;

		for (vector<FeaturePoint>::iterator it_2 = features.begin() ; it_2 != it_1 ; ++it_2)
		{
			if ((abs((*it_1).x - (*it_2).x)  <= 1.0 ) && (abs((*it_1).y - (*it_2).y)  <= 1.0 ))
			{
				is_max = false;
				break;
			}
		}

		if (is_max == true)
			selected_features->push_back(*it_1);
	}
}

// Compute e
void MyComputeErrorVector(const ImgFloat& simg1, const ImgFloat& simg2, const ImgFloat& sgradx, const ImgFloat& sgrady, float* ex, float* ey)
{
	*ex = 0;
	*ey = 0;

	const float* p1 = simg1.Begin();
	const float* p2 = simg2.Begin();
	const float* px = sgradx.Begin();
	const float* py = sgrady.Begin();

	while (p1 != simg1.End())
	{
		float diff = (*p1) - (*p2);
		*ex += diff * (*px);
		*ey += diff * (*py);

		p1++;
		p2++;
		px++;
		py++;
	}
}

// Interpolation sampling
float MyBilinearInterpolate(const ImgFloat& img, const float& x, const float& y)
{
	double u, v;

	if (x >= (img.Width() - 1))
		u = img.Width() - 1 - 1.0e-5;
	else if (x < 0)
		u = 0;
	else
		u = x;

	if (y >= (img.Height() - 1))
		v = img.Height() - 1 - 1.0e-5;
	else if (y < 0)
		v = 0;
	else
		v = y;

	double a = u - floor(u);
	double b = v - floor(v);

	int u_int = (int) u;
	int v_int = (int) v;

	double out 
		= (1-a)*(1-b)*img(u_int    , v_int    )
		+ a    *(1-b)*img(u_int + 1, v_int    )
		+ a    *b    *img(u_int + 1, v_int + 1)
		+ (1-a)*b    *img(u_int    , v_int + 1);

	return (float)out;
}


// Bilinear interpolation
void MyBilinearInterpolationExtract(const ImgFloat& img, float xx, float yy, int size, ImgFloat* simg2)
{
	simg2->Reset(size, size);

	int w = (size - 1) / 2;

	for (int y = 0 ; y < size ; y++)
	{
		for (int x = 0 ; x < size ; x++)
		{
			(*simg2)(x, y) = MyBilinearInterpolate(img, x + xx - w, y + yy - w);
		}
	}
}

void MyTrackFeature(const ImgGray& in_1, const ImgGray& in_2, const int win_size, const float sigma, vector<FeaturePoint>* features)
{
	const int maxiter = 10;
	const float minshift = 0.1;
	const int w = (win_size - 1) / 2;

	ImgFloat in_1f, in_2f;
	ImgFloat gradx, grady, gradxx, gradyy, gradxy;
	ImgFloat sgradx, sgrady, simg1, simg2;

	Convert(in_1, &in_1f);
	Convert(in_2, &in_2f);

	// Compute gradients
	MyGetGradient(in_1f, sigma, &gradx, &grady, &gradxx, &gradxy, &gradyy);
	MySumGradient(win_size, &gradxx, &gradxy, &gradyy);

	for (int i = 0 ; i < features->size() ; i++)
	{
		float dx_total = 0, dy_total = 0;

		FeaturePoint& f = (*features).at(i);

		if (((f.x - w) < 0) || ((f.x + w) > in_1.Width() - 1) || ((f.y - w) < 0) || ((f.y + w) > in_1.Height() - 1))
		{
			f.val = 0;
			continue;
		}

		// Compute Z matrix
		float gxx = MyBilinearInterpolate(gradxx, f.x, f.y);
		float gxy = MyBilinearInterpolate(gradxy, f.x, f.y);
		float gyy = MyBilinearInterpolate(gradyy, f.x, f.y);

		// Precompute e
		MyBilinearInterpolationExtract(in_1f, f.x, f.y, win_size, &simg1);
		MyBilinearInterpolationExtract(gradx, f.x, f.y, win_size, &sgradx);
		MyBilinearInterpolationExtract(grady, f.x, f.y, win_size, &sgrady);

		int j = 0;
		bool done = false;

		do
		{
			// Compute e
			float ex = 0, ey = 0;
			MyBilinearInterpolationExtract(in_2f, f.x + dx_total, f.y + dy_total, win_size, &simg2);
			MyComputeErrorVector(simg1, simg2, sgradx, sgrady, &ex, &ey);

			// Compute u
			float det = gxx * gyy - gxy * gxy;
			float dx = (gyy * ex - gxy * ey) / det;
			float dy = (gxx * ey - gxy * ex) / det;

			dx_total += dx;
			dy_total += dy;

			// Done or not
			done = (j > maxiter) || ((fabs(dx) < minshift) && (fabs(dy) < minshift));
			j++;
		}
		while (!done);

		f.x += dx_total;
		f.y += dy_total;
	}
}

// Mark features on input image
void MyMarkFeature(const vector<FeaturePoint>* feature, ImgBgr* img)
{
	Bgr marker;

	for (vector<FeaturePoint>::const_iterator it = feature->begin() ; it != feature->end() ; ++it)
		if (it->val != 0)
			if ((MyRound(it->x) >= 1) && (MyRound(it->x) <= img->Width() - 2) && (MyRound(it->y) >= 1) && (MyRound(it->y) <= img->Height() - 2))
			{
				(*img)(MyRound(it->x)    , MyRound(it->y)    ) = marker.RED;
				(*img)(MyRound(it->x) - 1, MyRound(it->y) - 1) = marker.RED;
				(*img)(MyRound(it->x)    , MyRound(it->y) - 1) = marker.RED;
				(*img)(MyRound(it->x) + 1, MyRound(it->y) - 1) = marker.RED;
				(*img)(MyRound(it->x) + 1, MyRound(it->y)    ) = marker.RED;
				(*img)(MyRound(it->x) + 1, MyRound(it->y) + 1) = marker.RED;
				(*img)(MyRound(it->x)    , MyRound(it->y) + 1) = marker.RED;
				(*img)(MyRound(it->x) - 1, MyRound(it->y) + 1) = marker.RED;
				(*img)(MyRound(it->x) - 1, MyRound(it->y)    ) = marker.RED;
			}
}

int main(int argc, const char* argv[], const char* envp[])
{
	// Check arguments
	if (argc != 6)
	{
		cerr << "number of arguments must be five!" << endl;
		cerr << "usage: homework filename-format first-frame last-frame sigma window-size" << endl;
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
		// Read command line
		int first_frame, last_frame;
		float sigma;
		int win_size;

		if (!(win_size = atoi(argv[5])))
		{
			cerr << "usage: homework filename-format first-frame last-frame sigma window-size" << endl;
			system("pause");
			exit(1);
		}

		if (!(sigma = atof(argv[4])))
		{
			cerr << "usage: homework filename-format first-frame last-frame sigma window-size" << endl;
			system("pause");
			exit(1);
		}

		if (!(last_frame = atoi(argv[3])))
		{
			cerr << "usage: homework filename-format first-frame last-frame sigma window-size" << endl;
			system("pause");
			exit(1);
		}

		if (!(first_frame = atoi(argv[2])))
		{
			cerr << "usage: homework filename-format first-frame last-frame sigma window-size" << endl;
			system("pause");
			exit(1);
		}

		// Load image
		vector<ImgBgr> img_0;
		vector<ImgGray> img_1;
		ImgBgr img_0_temp;
		ImgGray img_1_temp;

		CString filename_format = argv[1];
		CString filename;

		for (int i = first_frame ; i < last_frame + 1 ; i++)
		{
			filename.Format(filename_format, i);

			Load(MyRedirectPath(filename), &img_0_temp);
			if (((img_0_temp.Width() == 0)||(img_0_temp.Height() == 0))) 
			{
				cerr << "cannot open zero size image!" <<  endl;
				system("pause");
				exit(1);
			}

			Convert(img_0_temp, &img_1_temp);

			img_0.push_back(img_0_temp);
			img_1.push_back(img_1_temp);
		}

		// Detect features
		cout << "Detecting features...";

		vector<FeaturePoint> feature;
		MyDetectFeature(img_1.at(0), &feature);
		MyMarkFeature(&feature, &img_0.at(0));

		cout << "done" << endl;

		// Track features
		cout << "Tracking features..." << endl;

		for (int i = 1 ; i < img_0.size() ; i++)
		{
			int step_percent = 100 * i / img_0.size();
			cout << step_percent << "%";

			MyTrackFeature(img_1.at(i - 1), img_1.at(i), win_size, sigma, &feature);
			MyMarkFeature(&feature, &img_0.at(i));

			cout << "\r";
		}

		cout << "\r100%...done" << endl;

		// Display image in figure window
		Figure fig_first = Figure("First Image", img_0_temp.Width() * 0, img_0_temp.Height() * 0, true);
		fig_first.Draw(img_0.at(0));

		Figure fig_tracking = Figure("Feature Trafcking", img_0_temp.Width() * 1, img_0_temp.Height() * 0, true);
		for (vector<ImgBgr>::iterator it = img_0.begin() ; it != img_0.end() ; it++)
		{
			fig_tracking.Draw(*it);
			Sleep(100);
		}

		//// Loop forever until user presses Ctrl-C in terminal window.
		EventLoop();
	}
	catch (const Exception& e) 
	{
		e.Display();    // display exception to user in a popup window 
	}

	return 0;
}