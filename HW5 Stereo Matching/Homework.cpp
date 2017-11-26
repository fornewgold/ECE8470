/*
ECE 847
Assignment #5

This code is to implement efficient sliding window (block-based) matching of two rectified stereo images.    

usage: homework left-filename right-filename max-disparity

Xi Zhao
11/16/2013
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

const int min_disparity = 0;
const int block_size = 7;

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

// Compute disparity map
void MyBlookMatch(ImgGray & in_left, ImgGray & in_right, const int min, const int max, const int size, ImgInt * out)
{
	out->Reset(in_left.Width(), in_left.Height());
	Set(out, 0);

	int g;
	int g_hat;
	int d_hat;
	int sad;
	int w = (size - 1) / 2;
	int x_left, y_left;
	int x_right, y_right;
	int count, miss_count;

	for	(int y = 0 ; y < in_left.Height() ; y++)
	{
		for (int x = 0 ; x < in_left.Width() ; x++)
		{
			g_hat = in_left.Height() * in_left.Width() + 1;

			for (int d = min ; d < max + 1 ; d++)
			{
				g = 0;
				count = 0;
				miss_count = 0;

				for (int j = - w ; j < w + 1 ; j++)
				{
					for (int i = - w ; i < w + 1 ; i ++)
					{
						x_left = x + i;
						y_left = y + j;
						x_right = x + i - d;
						y_right = y + j;

						if ((x_right < 0)||(x_left > in_left.Width() - 1)||(y_left < 0)||(y_left > in_left.Height() - 1))
						{
							miss_count++;
							continue;
						}
						else
						{
							count++;
							sad = abs(in_left(x + i, y + j) - in_right(x + i - d, y + j));
						}

						g += sad;
					}
				}

				g = MyRound(g * ((float) (count + miss_count) / (float) (count + 1)));

				if (g < g_hat)
				{
					g_hat = g;
					d_hat = d;
				}
			}

			(*out)(x, y) = d_hat;
		}
	}

	Figure disparity_map = Figure("Disparity Map", in_left.Width() * 0, in_left.Height() * 1, true);
	disparity_map.Draw(*out);
}

// Precompute dissimilarity
void MySummedDissimilarity(ImgGray & in_left, ImgGray & in_right, const int d_size, const int box_size, float * d_sum_cube)
{
	int cube_size = in_left.Width() * in_left.Height() * d_size;
	int w_half = (box_size - 1) / 2;

	// Compute disparity
	float * dissimilarity_cube = NULL;
	dissimilarity_cube = new float[cube_size];

	for (int d = 0 ; d < d_size ; d++)
	{
		for	(int y = 0 ; y < in_left.Height() ; y++)
		{
			for (int x = 0 ; x < in_left.Width() ; x++)
			{
				if (((x - d) < 0)|| ((x - d) > in_left.Width() - 1))
					dissimilarity_cube[d * in_left.Width() * in_left.Height() + y * in_left.Width() + x] = 0;
				else
					dissimilarity_cube[d * in_left.Width() * in_left.Height() + y * in_left.Width() + x] = (float) abs(in_left(x, y) - in_right(x - d, y));
			}
		}
	}

	// Convolve box filter
	float * temp_cube = NULL;
	temp_cube = new float[cube_size];

	float * box = NULL;
	box = new float[box_size];

	for (int i = 0 ; i < box_size ; i++)
		box[i] = (float) 1.0 / box_size;

	for (int d = 0 ; d < d_size ; d++)
	{
		// Convolve vetically
		for (int x = 0 ; x < in_left.Width() ; x++)
		{
			for (int y = 0 ; y < in_left.Height() ; y++)
			{
				float y_val = 0;

				for (int i = 0 ; i < box_size ; i++)
				{
					int y_hat = y + w_half - i;

					if ((y_hat >= 0) && (y_hat <= (in_left.Height() - 1)))
						y_val += box[i] * dissimilarity_cube[d * in_left.Width() * in_left.Height() + y_hat * in_left.Width() + x];
					else if (y_hat < 0)
						y_val += box[i] * dissimilarity_cube[d * in_left.Width() * in_left.Height() + 0 * in_left.Width() + x];
					else 
						y_val += box[i] * dissimilarity_cube[d * in_left.Width() * in_left.Height() + (in_left.Height() - 1) * in_left.Width() + x];
				}

				temp_cube[d * in_left.Width() * in_left.Height() + y * in_left.Width() + x] = y_val;
			}
		}

		// Convolve horizontally
		for (int y = 0 ; y < in_left.Height() ; y++)
		{
			for (int x = 0 ; x < in_left.Width() ; x++)
			{
				float x_val = 0;

				for (int i = 0 ; i < box_size ; i++)
				{
					int x_hat = x + w_half - i;

					if ((x_hat >= 0) && (x_hat <= (in_left.Width() - 1)))
						x_val += box[i] * temp_cube[d * in_left.Width() * in_left.Height() + y * in_left.Width() + x_hat];
					else if (x_hat < 0)
						x_val += box[i] * temp_cube[d * in_left.Width() * in_left.Height() + y * in_left.Width() + 0];
					else
						x_val += box[i] * temp_cube[d * in_left.Width() * in_left.Height() + y * in_left.Width() + (in_left.Width() - 1)];
				}

				d_sum_cube[d * in_left.Width() * in_left.Height() + y * in_left.Width() + x] = x_val;
			}
		}
	}
}  

// Compute disparity map with precomputed summation
void MyQuickBloodMatch(ImgGray & in_left, ImgGray & in_right, const int min, const int max, const int block_size, ImgInt * out, ImgInt * out_check)
{
	out->Reset(in_left.Width(), in_left.Height());
	Set(out, 0);
	out_check->Reset(in_left.Width(), in_left.Height());
	Set(out, 0);

	int d_size = max - min + 1;
	int cube_size = in_left.Width() * in_left.Height() * d_size;

	float * dissimilarity_cube = NULL;
	dissimilarity_cube = new float[cube_size];

	MySummedDissimilarity(in_left, in_right, d_size, block_size, dissimilarity_cube);

	for (int x = 0 ; x < in_left.Width() ; x++)
	{
		for (int y = 0 ; y < in_left.Height() ; y++)
		{
			int d_l_hat = 0;
			int d_r_hat = 0;

			float g_l_hat = (float) in_left.Width() * in_left.Height() + 1;											
			float g_r_hat = (float) in_left.Width() * in_left.Height() + 1;

			// Left
			{
				for (int d = 0 ; d < d_size ; d++)
				{
					if (dissimilarity_cube[d * in_left.Width() * in_left.Height() + y * in_left.Width() + x] < g_l_hat)
					{
						g_l_hat = dissimilarity_cube[d * in_left.Width() * in_left.Height() + y * in_left.Width() + x];
						d_l_hat = d;
					}
				}
			}

			// right
			{
				for (int d = 0 ; d < d_size ; d++)
				{
					if (dissimilarity_cube[d * in_left.Width() * in_left.Height() + y * in_left.Width() + x] < g_r_hat)
					{
						g_r_hat = dissimilarity_cube[d * in_left.Width() * in_left.Height() + y * in_left.Width() + (x - d_l_hat + d)];
						d_r_hat = d;
					}
				}
			}

			(*out)(x, y) = d_l_hat;
			(*out_check)(x, y) = (d_l_hat == d_r_hat) ? d_l_hat : -1;
		}
	}

	Figure disparity_map = Figure("Left Disparity Without Check", in_left.Width() * 0, in_left.Height() * 1, true);
	disparity_map.Draw(*out);

	Figure disparity_map_check = Figure("Left Disparity With Check", in_left.Width() * 1, in_left.Height() * 1, true);
	disparity_map_check.Draw(*out_check);
}

// Output PLY file
void MyWritePLY(const ImgBgr & in, const ImgInt & disparity)
{
	long num_valid = 0;
	int red;
	int green;
	int blue;
	float x_val;
	float y_val;
	float z_val;
	float k = 1000;

	blepo::ImgInt::ConstIterator p;
	for (p = disparity.Begin() ; p != disparity.End() ; p++)
		if ((*p) != -1)
			num_valid++;

	ofstream output_ply;
	output_ply.open("output.ply", std::ofstream::binary | std::ofstream::trunc);

	output_ply << "ply\r\n";
	output_ply << "format ascii 1.0\r\n";
	output_ply << "element vertex " << num_valid <<"\r\n";
	output_ply << "property float x\r\n";
	output_ply << "property float y\r\n";
	output_ply << "property float z\r\n";
	output_ply << "property uchar diffuse_red\r\n";
	output_ply << "property uchar diffuse_green\r\n";
	output_ply << "property uchar diffuse_blue\r\n";
	output_ply << "end_header\r\n";

	for (int x = 0 ; x < in.Width() ; x++)
	{
		for (int y = 0 ; y < in.Height() ; y++)
		{
			if (disparity(x, y) == -1)
				continue;
			else 
			{
				x_val = (float) x;
				y_val = (float) y;
				z_val = (disparity(x, y) == 0) ? (k / 1) : (k / disparity(x, y));
				red = in(x, y).r;
				green = in(x, y).g;
				blue = in(x, y).b;
			}

			output_ply << x_val << " " << y_val << " " << z_val << " " << red << " " << green << " " << blue << "\r\n";
		}
	}

	output_ply.close();
}

int main(int argc, const char* argv[], const char* envp[])
{
	// Check arguments
	if (argc != 4)
	{
		cerr << "number of arguments must be three!" << endl;
		cerr << "usage: homework left-filename right-filename max-disparity" << endl;
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
		int max_disparity;

		if (!(max_disparity = atoi(argv[3])))
		{
			cerr << "usage: homework left-filename right-filename max-disparity" << endl;
			system("pause");
			exit(1);
		}

		// Load image
		ImgBgr img_left_0;
		ImgBgr img_right_0;

		Load(MyRedirectPath(argv[1]), &img_left_0);
		if (((img_left_0.Width() == 0)||(img_left_0.Height() == 0))) 
		{
			cerr << "cannot open zero size image!" <<  endl;
			system("pause");
			exit(1);
		}

		Load(MyRedirectPath(argv[2]), &img_right_0);
		if (((img_right_0.Width() == 0)||(img_right_0.Height() == 0))) 
		{
			cerr << "cannot open zero size image!" <<  endl;
			system("pause");
			exit(1);
		}

		if (!((img_left_0.Width() == img_right_0.Width())&&(img_left_0.Height() == img_right_0.Height())))
		{
			cerr << "sizes of input images are not same!" <<  endl;
			system("pause");
			exit(1);

		}

		Figure fig_left = Figure("Left Image", img_left_0.Width() * 0, img_right_0.Height() * 0, true);
		fig_left.Draw(img_left_0);

		Figure fig_right = Figure("Right Image", img_left_0.Width() * 1, img_right_0.Height() * 0, true);
		fig_right.Draw(img_right_0);

		// Convert input images to grayscale
		ImgGray img_left_1;
		ImgGray img_right_1;

		Convert(img_left_0, &img_left_1);
		Convert(img_right_0, &img_right_1);

		// Match block
		ImgInt disparity_map; 
		ImgInt disparity_map_check; 

		//MyBlookMatch(img_left_1, img_right_1, min_disparity, max_disparity, block_size, &disparity_map);
		MyQuickBloodMatch(img_left_1, img_right_1, min_disparity, max_disparity, block_size, &disparity_map, &disparity_map_check);

		// Write ply file
		MyWritePLY(img_left_0, disparity_map_check);

		// Loop forever until user presses Ctrl-C in terminal window.
		EventLoop();
	}
	catch (const Exception& e) 
	{
		e.Display();    // display exception to user in a popup window 
	}

	return 0;
}