
/*
 * $Id: motion_detect.c,v 1.5 2008/06/30 13:57:16 bnv Exp $
 * $Log: motion_detect.c,v $
 * Revision 1.5  2008/06/30 13:57:16  bnv
 * Added option as camera width and height
 *
 * Revision 1.4  2007/12/09 13:25:42  bnv
 * Changed to use V4L with YUV420P (only) for the moment
 *
 * Revision 1.3  2006/09/04 07:49:35  bnv
 * Using the new version of dc1394 library
 *
 * Revision 1.2  2006/01/10 15:24:32  bnv
 * Added: JPEG exporting
 *
 * Revision 1.1  2006/01/10 14:13:32  bnv
 * Initial revision
 *
 *
 * Author:	Vasilis.Vlachoudis@cern.ch
 * Date:	28-Jan-2005
 */

#include <os.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>
#include <time.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <linux/videodev.h>

#define VIDEODEV "/dev/video"

/*
 * Include file for users of JPEG library.
 * You will need to have included system headers that define at least
 * the typedefs FILE and size_t before you can include jpeglib.h.
 * (stdio.h is sufficient on ANSI-conforming systems.)
 * You may also wish to include "jerror.h".
 */
#include "jpeglib.h"
#include "jerror.h"
#include <setjmp.h>

#define MAX_PORTS 4
#define MAX_RESETS 10

int DrawText(byte *image, int width, int height, char *txt, int x, int y, int col);

struct video_capability	cap;
struct video_window	win;
struct video_picture	vpic;

double	threshold    = 200.0;
int	time2sleep   = 5;
int	totalTime    = 86400;
int	cameraWidth  = -1;
int	cameraHeight = -1;

char *g_filename = "image-%08d.jpg";

static struct option long_options[] = {
	{"dev", 1, NULL, 0},
	{"threshold", 1, NULL, 0},
	{"sleep", 1, NULL, 0},
	{"time", 1, NULL, 0},
	{"help", 0, NULL, 0},
	{"width", 1, NULL, 0},
	{"height", 1, NULL, 0},
	{NULL, 0, 0, 0}
};

/* --- get_options --- */
void get_options(int argc, char *argv[])
{
	int option_index = 0;

	while (getopt_long(argc, argv, "", long_options, &option_index) >= 0) {
		switch (option_index) {
			/* case values must match long_options */
			case 1:
				sscanf(optarg, "%lg", &threshold);
				break;
			case 2:
				sscanf(optarg, "%d", &time2sleep);
				break;
			case 3:
				sscanf(optarg, "%d", &totalTime);
				break;
			case 4:
				sscanf(optarg, "%d", &cameraWidth);
				break;
			case 5:
				sscanf(optarg, "%d", &cameraHeight);
				break;
			default:
				printf("\n"
				       "%s - grab a color sequence using format0, rgb mode\n\n"
				       "Usage:\n"
				       "    %s [--dev=/dev/video0]\n\n"
				       "    --threshold - specifies RMS threshold to use (default: 200)\n"
				       "    --sleep     - specifies seconds to sleep (default: 5)\n"
				       "    --time      - specifies seconds to run (default: 86400)\n"
				       "    --width #   - camera width to use\n"
				       "    --height #  - camera height to use\n"
				       "    --help      - prints this message\n"
				       "    filename is optional; the default is image%%08d.ppm\n\n",
				       argv[0], argv[0]);
				exit(0);
		}
	}
	if (optind < argc)
		g_filename = argv[optind];

	printf("Options\n");
	printf("\tThreshold\t%g\n", threshold);
	printf("\tSleep Time\t%d s\n", time2sleep);
	printf("\tFilename\t%s\n\n", g_filename);
} /* get_options */

/* --- compareFrames --- */
int compareFrames(byte *frame, byte *prev_frame)
{
	int	i;
	double	sum2 = 0.0;
	double	rms;

	for (i=0; i<win.width*win.height; i++) {
		double	currGray, prevGray;
		int	red, green, blue;
		int	diff;

		red   = *frame++;
		green = *frame++;
		blue  = *frame++;
		currGray = 0.3  * (double)red +
			   0.59 * (double)green +
			   0.11 * (double)blue;

		red   = *prev_frame++;
		green = *prev_frame++;
		blue  = *prev_frame++;
		prevGray = 0.3  * (double)red +
			   0.59 * (double)green +
			   0.11 * (double)blue;

		diff = currGray-prevGray;
		sum2 += diff*diff;
	}

	rms = sum2/(double)(win.height*win.width);
	printf("SUM2=%lg RMS=%lg\n", sum2, rms);

	return (rms>threshold);
} /* compareFrames */

/* --- timeStamp --- */
long timeStamp()
{
	time_t	now;
	struct tm *tmdata ;

	now = time(NULL);
	tmdata = localtime(&now) ;
	return tmdata->tm_yday*86400 +
		((tmdata->tm_hour*60L)+tmdata->tm_min)*60L +
		(long)tmdata->tm_sec;
} /* timeStamp */

/* --- writeJPEGFile --- */
GLOBAL(void) writeJPEGFile(char *filename, int quality, byte *frame)
{
	/* This struct contains the JPEG compression parameters and pointers to
	 * working space (which is allocated as needed by the JPEG library).
	 * It is possible to have several such structures, representing multiple
	 * compression/decompression processes, in existence at once.  We refer
	 * to any one struct (and its associated working data) as a "JPEG object".
	 */
	struct jpeg_compress_struct cinfo;
	/* This struct represents a JPEG error handler.  It is declared separately
	 * because applications often want to supply a specialized error handler
	 * (see the second half of this file for an example).  But here we just
	 * take the easy way out and use the standard error handler, which will
	 * print a message on stderr and call exit() if compression fails.
	 * Note that this struct must live as long as the main JPEG parameter
	 * struct, to avoid dangling-pointer problems.
	 */
	struct jpeg_error_mgr jerr;
	/* More stuff */
	FILE * outfile;		/* target file */
	JSAMPROW row_pointer[1];	/* pointer to JSAMPLE row[s] */
	int row_stride;		/* physical row width in image buffer */

	/* Step 1: allocate and initialize JPEG compression object */

	/* We have to set up the error handler first, in case the initialization
	 * step fails.  (Unlikely, but it could happen if you are out of memory.)
	 * This routine fills in the contents of struct jerr, and returns jerr's
	 * address which we place into the link field in cinfo.
	 */
	cinfo.err = jpeg_std_error(&jerr);
	/* Now we can initialize the JPEG compression object. */
	jpeg_create_compress(&cinfo);

	/* Step 2: specify data destination (eg, a file) */
	/* Note: steps 2 and 3 can be done in either order. */

	/* Here we use the library-supplied code to send compressed data to a
	 * stdio stream.  You can also write your own code to do something else.
	 * VERY IMPORTANT: use "b" option to fopen() if you are on a machine that
	 * requires it in order to write binary files.
	 */
	if ((outfile = fopen(filename, "wb")) == NULL) {
		fprintf(stderr, "can't open %s\n", filename);
		exit(1);
	}
	jpeg_stdio_dest(&cinfo, outfile);

	/* Step 3: set parameters for compression */

	/* First we supply a description of the input image.
	 * Four fields of the cinfo struct must be filled in:
	 */
	/* image width and height, in pixels */
	cinfo.image_width      = win.width;
	cinfo.image_height     = win.height;
	cinfo.input_components = 3;		/* # of color components per pixel */
	cinfo.in_color_space = JCS_RGB;	/* colorspace of input image */
	/* Now use the library's routine to set default compression parameters.
	 * (You must set at least cinfo.in_color_space before calling this,
	 * since the defaults depend on the source color space.)
	 */
	jpeg_set_defaults(&cinfo);
	/* Now you can set any non-default parameters you wish to.
	 * Here we just illustrate the use of quality (quantization table) scaling:
	 */
	jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);

	/* Step 4: Start compressor */

	/* TRUE ensures that we will write a complete interchange-JPEG file.
	 * Pass TRUE unless you are very sure of what you're doing.
	 */
	jpeg_start_compress(&cinfo, TRUE);

	/* Step 5: while (scan lines remain to be written) */
	/*           jpeg_write_scanlines(...); */

	/* Here we use the library's state variable cinfo.next_scanline as the
	 * loop counter, so that we don't have to keep track ourselves.
	 * To keep things simple, we pass one scanline per call; you can pass
	 * more if you wish, though.
	 */
	row_stride = win.width * 3;	/* JSAMPLEs per row in image_buffer */

	while (cinfo.next_scanline < cinfo.image_height) {
		/* jpeg_write_scanlines expects an array of pointers to scanlines.
		 * Here the array is only one element long, but you could pass
		 * more than one scanline at a time if that's more convenient.
		 */
		row_pointer[0] = & (frame)[cinfo.next_scanline * row_stride];
		(void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}

	/* Step 6: Finish compression */

	jpeg_finish_compress(&cinfo);
	/* After finish_compress, we can close the output file. */
	fclose(outfile);

	/* Step 7: release JPEG compression object */

	/* This is an important step since it will release a good deal of memory. */
	jpeg_destroy_compress(&cinfo);

	printf("wrote: %s\n", filename);

	/* And we're done! */
} /* writeJPEGFile */

/* --- saveImage --- */
int saveImage(byte *frame)
{
	char	filename[128];
	char	str[256];
	time_t	now;
	struct tm *tmdata ;
	int	xt, yt;

	now = time(NULL);
	tmdata = localtime(&now) ;
	sprintf(str,"%d/%d/%d %d:%02d:%02d",
			tmdata->tm_mday,
			tmdata->tm_mon+1,
			tmdata->tm_year+1900,
			tmdata->tm_hour,
			tmdata->tm_min,
			tmdata->tm_sec);

	xt = 5;
	yt = win.height-24;
	DrawText(frame, win.width, win.height, str, xt, yt, 0x1F1F1F);
	DrawText(frame, win.width, win.height, str, xt-2, yt-2, 0xFFFF7F);

	sprintf(filename, g_filename, timeStamp());
	writeJPEGFile(filename, 80, frame);
	return 0;
} /* saveImage */

#define READ_VIDEO_PIXEL(buf, format, depth, r, g, b)                   \
{                                                                       \
	switch (format)                                                 \
	{                                                               \
		case VIDEO_PALETTE_GREY:                                \
			switch (depth)                                  \
			{                                               \
				case 4:                                 \
				case 6:                                 \
				case 8:                                 \
					(r) = (g) = (b) = (*buf++ << 8);\
					break;                          \
									\
				case 16:                                \
					(r) = (g) = (b) =               \
						*((unsigned short *) buf);      \
					buf += 2;                       \
					break;                          \
			}                                               \
			break;                                          \
									\
									\
		case VIDEO_PALETTE_RGB565:                              \
		{                                                       \
			unsigned short tmp = *(unsigned short *)buf;    \
			(r) = tmp&0xF800;                               \
			(g) = (tmp<<5)&0xFC00;                          \
			(b) = (tmp<<11)&0xF800;                         \
			buf += 2;                                       \
		}                                                       \
		break;                                                  \
									\
		case VIDEO_PALETTE_RGB555:                              \
			(r) = (buf[0]&0xF8)<<8;                         \
			(g) = ((buf[0] << 5 | buf[1] >> 3)&0xF8)<<8;    \
			(b) = ((buf[1] << 2 ) & 0xF8)<<8;               \
			buf += 2;                                       \
			break;                                          \
									\
		case VIDEO_PALETTE_RGB24:                               \
			(r) = buf[0] << 8; (g) = buf[1] << 8;           \
			(b) = buf[2] << 8;                              \
			buf += 3;                                       \
			break;                                          \
									\
		default:                                                \
			fprintf(stderr,                                 \
				"Format %d not yet supported\n",        \
				format);                                \
	}                                                               \
}
/* --- yuv2rgb --- */
static void yuv2rgb(int y, int u, int v, int *r, int *g, int *b)
{
	*r = y + (1.370705 * (v-128));
	*g = y - (0.698001 * (v-128)) - (0.337633 * (u-128));
	*b = y + (1.732446 * (u-128));

	/* Even with proper conversion, some values still need clipping. */
	if (*r > 255) *r = 255;
	if (*g > 255) *g = 255;
	if (*b > 255) *b = 255;
	if (*r < 0) *r = 0;
	if (*g < 0) *g = 0;
	if (*b < 0) *b = 0;

	/* Values only go from 0-220..  Why? */
	*r = (*r * 220) / 256;
	*g = (*g * 220) / 256;
	*b = (*b * 220) / 256;
} /* yuv2rgb */

/* --- is_yuv --- */
static int is_yuv(uint16_t palette)
{
	return ((palette == VIDEO_PALETTE_YUV422)  ||
		(palette == VIDEO_PALETTE_YUYV)    ||
		(palette == VIDEO_PALETTE_UYVY)    ||
		(palette == VIDEO_PALETTE_YUV420)  ||
		(palette == VIDEO_PALETTE_YUV411)  ||
		(palette == VIDEO_PALETTE_YUV422P) ||
		(palette == VIDEO_PALETTE_YUV411P) ||
		(palette == VIDEO_PALETTE_YUV420P) ||
		(palette == VIDEO_PALETTE_YUV410P));
} /* is_yuv */

/* --- capture --- */
int capture(int fd, byte *buffer, byte *frame)
{
	int	i;
	byte	*src, *dst;
	int	y, u, v, r, g, b;
	unsigned src_depth = vpic.depth;

	// adjust brightness when told so and using some RGB palette
//	i = 0;
	//do {
	if (read(fd, buffer, (win.width * win.height * src_depth)/8)<0)
		return TRUE;

	src = buffer;
	dst = frame;
	for (i=0; i<win.width*win.height; i++) {
		/*
		if (!is_yuv(vpic.palette)) {
			READ_VIDEO_PIXEL(src, vpic.palette, src_depth, r, g, b);
			*src++ = r;
			*src++ = g;
			*src++ = b;
		}
		else if (vpic.palette == VIDEO_PALETTE_UYVY) {
			if(i==0) v = src[3];
			if((i%2) == 0) {
				u = src[0];
				y = src[1];
				src += 2;		// v from next 16bit word
			} else {
				v = src[0];
				y = src[1];
				src += 2;
			}
		}
		else if ((vpic.palette == VIDEO_PALETTE_YUYV) ||
			  (vpic.palette == VIDEO_PALETTE_YUV422)) {
			if(i==0)
				v = src[4];
			if((i%2) == 0) {
				y = src[0];
				u = src[1];
				src += 2;		// v from next 16bit word
			} else {
				y = src[0];
				v = src[1];
				src += 2;
			}
		}
		else if (vpic.palette == VIDEO_PALETTE_YUV422P) {
			y = buffer[i];
			if (i == 0)
				src = buffer + (win.width*win.height);
			u = src[(i/2)%(win.width/2)];
			v = src[(win.width*win.height)/2 + (i/2)%(win.width/2)];
			if (i && (i%(win.width*2) == 0))
				src += win.width;
		}
		else if(vpic.palette == VIDEO_PALETTE_YUV411P) {
			y = buffer[i];
			if (i == 0)
				src = buffer + (win.width*win.height);
			u = src[(i/4)%(win.width/4)];
			v = src[(win.width*win.height)/4 + (i/4)%(win.width/4)];
			if (i && (i%(win.width) == 0))
				src += win.width/4;
		}
		else
		*/
		if((vpic.palette == VIDEO_PALETTE_YUV420P) ||
			  (vpic.palette == VIDEO_PALETTE_YUV420)) {
			if (i == 0)
				src = buffer + (win.width*win.height);
			y = buffer[i];
			u = src[(i/2)%(win.width/2)];
			v = src[(win.width*win.height)/4 + (i/2)%(win.width/2)];
			if (i && (i%(win.width*4)) == 0)
				src += win.width;
		}
		/*
		else if(vpic.palette == VIDEO_PALETTE_YUV410P) {
			if (i == 0)
				src = buffer + (win.width*win.height);
			y = buffer[i];
			u = src[(i/4)%(win.width/4)];
			v = src[(win.width*win.height)/16 + (i/4)%(win.width/4)];
			if (i && (i%(win.width*4)) == 0)
				src += win.width/4;
		}
		*/
		yuv2rgb(y, u, v, &r, &g, &b);
		*dst++ = r;
		*dst++ = g;
		*dst++ = b;
	}
	return 0;
} /* capture */

/* --- main --- */
int main(int argc, char *argv[])
{
	long	startTime;
	int	fd;
	byte	*buffer, *frame, *prev_frame;

	get_options(argc, argv);

	fd = open(VIDEODEV, O_RDONLY);
	if (fd < 0) {
		perror(VIDEODEV);
		exit(1);
	}

	if (ioctl(fd, VIDIOCGCAP, &cap) < 0) {
		perror("VIDIOGCAP");
		fprintf(stderr, "(" VIDEODEV " not a video4linux device?)\n");
		close(fd);
		exit(1);
	}

	if (ioctl(fd, VIDIOCGWIN, &win) < 0) {
		perror("VIDIOCGWIN");
		close(fd);
		exit(1);
	}

	if (ioctl(fd, VIDIOCGPICT, &vpic) < 0) {
		perror("VIDIOCGPICT");
		close(fd);
		exit(1);
	}


	// Check for video capture
	if (!(cap.type & VID_TYPE_CAPTURE)) {
		fprintf(stderr, "error: video4linux device %s cannot capture to memory\n",
			VIDEODEV);
		exit(1);
	}

	// Use maximum resolution
	if (cameraWidth < 0)
		win.width  = cap.maxwidth;
	else
		win.width  = cameraWidth;

	if (cameraHeight < 0)
		win.height = cap.maxheight;
	else
		win.height = cameraHeight;

	if (ioctl(fd, VIDIOCSWIN, &win) < 0) {
		perror("VIDIOCSWIN");
		close(fd);
		exit(1);
	}

	buffer     = malloc(win.width * win.height * vpic.depth/8);
	frame      = malloc(win.width * win.height * 3);	// RGB
	prev_frame = malloc(win.width * win.height * 3);	// RGB
	if (!buffer || !frame || !prev_frame) {
		fprintf(stderr, "Out of memory.\n");
		exit(1);
	}

	/*-----------------------------------------------------------------------
	*  capture one frame
	*-----------------------------------------------------------------------*/
	if (capture(fd, buffer, frame)) {
		fprintf( stderr, "unable to capture a frame\n");
		close(fd);
		exit(2);
	}

	if (saveImage(frame)) {
		perror("Can't create output file");
		close(fd);
		exit(2);
	}

	startTime = timeStamp();
	while (timeStamp()-startTime<=totalTime) {
		/*----------------------------------------------------------
		 *  copy image
		 *----------------------------------------------------------*/
		memcpy(prev_frame, frame, win.width * win.height * 3);
		sleep(time2sleep);

		/*----------------------------------------------------------
		 *  capture next image
		 *----------------------------------------------------------*/
		if (capture(fd, buffer, frame)) {
			fprintf( stderr, "unable to capture a frame\n");
			close(fd);
			exit(2);
		}

		/*---------------------------------------------------------
		 *  compare and save
		 *---------------------------------------------------------*/
		if (compareFrames(frame, prev_frame))
			if (saveImage(frame)) {
				perror("Can't create output file");
				close(fd);
				exit(2);
			}
	}

	/*-----------------------------------------------------------------------
	 *  Close camera
	 *-----------------------------------------------------------------------*/
	 close(fd);
	 return 0;
} /* main */
