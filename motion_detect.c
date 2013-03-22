
/*
 * $Id: motion_detect.c,v 3.1 2013/03/22 09:21:39 bnv Exp $
 * $Log: motion_detect.c,v $
 * Revision 3.1  2013/03/22 09:21:39  bnv
 * Added printout of total time
 *
 * Revision 3.0  2013/03/22 09:18:01  bnv
 * Moved to RGB formating and using the v4l2 decoding
 *
 * Revision 2.4  2013/03/22 08:17:53  bnv
 * Print out of information
 *
 * Revision 2.3  2013/03/14 08:28:21  bnv
 * Help corrected
 *
 * Revision 2.2  2013/03/08 16:35:25  bnv
 * Corrected YUV420 and reading buffer
 *
 * Revision 2.1  2011/05/05 09:08:47  bnv
 * Minor change
 *
 * Revision 2.0  2011/05/05 08:56:23  bnv
 * First version working with V4L2
 *
 * Revision 1.6  2011/05/05 07:59:57  bnv
 * Last working version
 *
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
 * Author:	Vasilis.Vlachoudis@cern.ch
 * Date:	28-Jan-2005
 */

#include <os.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>
#include <time.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <sys/mman.h>
#include <linux/videodev2.h>
#include "libv4l2.h"

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

char   videodev[128];

struct buffer {
        void   *start;
        size_t length;
};

struct v4l2_capability	cap;
struct v4l2_format	fmt;
struct v4l2_buffer	buf;
struct v4l2_requestbuffers	req;
enum v4l2_buf_type	type;
struct buffer		*buffers;
int			n_buffers;

double	threshold          =  200.0;
int	totalTime          =  86400;
dword	cameraFormat       =  0;
int	cameraWidth        =  0;
int	cameraHeight       =  0;
int	cameraSize         =  0;
int	cameraBytesperline =  0;
int	stop               =  0;
int	verbose            =  0;
useconds_t	time2sleep =  1000000;

char *g_filename = "image-%05d.jpg";

static const char short_options [] = "d:h:m:r:s:T:t:w:v?";
static struct option long_options[] = {
	{"device",    required_argument, NULL, 'd'} ,
	{"height",    required_argument, NULL, 'h'} ,
	{"help",      no_argument,       NULL, '?'} ,
	{"sleep",     required_argument, NULL, 's'} ,
	{"threshold", required_argument, NULL, 'T'} ,
	{"time",      required_argument, NULL, 't'} ,
	{"verbose",   no_argument,       NULL, 'v'} ,
	{"width",     required_argument, NULL, 'w'} ,
	{NULL, 0, 0, 0}
};

static void sighup_handler(int s)
{
	stop = 1;
} /* sighup_handler */

/* --- xioctl --- */
static int xioctl(int fh, int request, void *arg)
{
	int r;
	do {
		r = v4l2_ioctl(fh, request, arg);
	} while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

	if (r == -1) {
		perror("xioctl");
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
	return r;
} /* xioctl */

/* --- compareFrames --- */
int compareFrames(byte *frame, byte *prev_frame)
{
	int	i;
	double	sum2 = 0.0;
	double	rms;

	for (i=0; i<cameraWidth*cameraHeight; i++) {
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

	rms = sum2/(double)(cameraHeight*cameraWidth);
	if (verbose)
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
	cinfo.image_width      = cameraWidth;
	cinfo.image_height     = cameraHeight;
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
	row_stride = cameraWidth * 3;	/* JSAMPLEs per row in image_buffer */

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
static	int	idx = 0;

	now = time(NULL);
	tmdata = localtime(&now) ;
	sprintf(str,"%02d.%02d.%04d %d:%02d:%02d",
			tmdata->tm_mday,
			tmdata->tm_mon+1,
			tmdata->tm_year+1900,
			tmdata->tm_hour,
			tmdata->tm_min,
			tmdata->tm_sec);

	xt = 5;
	yt = cameraHeight-24;
	DrawText(frame, cameraWidth, cameraHeight, str, xt, yt, 0x1F1F1F);
	DrawText(frame, cameraWidth, cameraHeight, str, xt-2, yt-2, 0xFFFF7F);

	sprintf(filename, g_filename, idx++);
	writeJPEGFile(filename, 80, frame);
	return 0;
} /* saveImage */

/* --- capture --- */
int capture(int fd, byte *frame)
{
	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	xioctl(fd, VIDIOC_DQBUF, &buf);

	memcpy(frame, buffers[buf.index].start, buf.bytesused);
	return xioctl(fd, VIDIOC_QBUF, &buf);
} /* capture */

/* --- get_options --- */
void get_options(int argc, char *argv[])
{
	int c;
	int option_index = 0;
	double sleeps;

	strcpy(videodev, "/dev/video0");
	while ((c=getopt_long(argc, argv,
			short_options, long_options, &option_index)) >= 0) {
                switch (c) {
			/* case values must match long_options */
			case 'd':
				strcpy(videodev, optarg);
				break;
			case 'T':
				threshold = atof(optarg);
				break;
			case 's':
				sleeps = atof(optarg);
				time2sleep = (useconds_t)(sleeps*1.0e6);
				break;
			case 't':
				totalTime = atoi(optarg);
				break;
			case 'w':
				cameraWidth = atoi(optarg);
				break;
			case 'h':
				cameraHeight = atoi(optarg);
				break;
			case 'v':
				verbose = 1;
				break;

			default:
				printf("\n"
				       "%s - grab a color sequence using format0, rgb mode\n\n"
				       "Usage:\n"
				       "    %s [--device=/dev/video0]\n\n"
				       "    -d --device #  - specify the video device to use (default: /dev/video0)\n"
				       "    -T --threshold - specifies RMS threshold to use (default: 200)\n"
				       "    -s --sleep     - specifies seconds to sleep (default: 1.0)\n"
				       "    -t --time      - specifies seconds to run (default: 86400)\n"
				       "    -w --width #   - camera width to use\n"
				       "    -h --height #  - camera height to use\n"
				       "    -v --verbose   - be verbose\n"
				       "    -? --help      - prints this message\n"
				       "    filename is optional; the default is image%%08d.ppm\n"
				       "Send SIGHUP to stop smoothly\n\n",
				       argv[0], argv[0]);
				exit(0);
		}
	}
	if (optind < argc)
		g_filename = argv[optind];

	printf("Options\n");
	printf("\tThreshold\t%g\n", threshold);
	printf("\tSleep Time\t%g s\n", (double)time2sleep/1.0E6);
	printf("\tRun Time\t%d s\n", totalTime);
	printf("\tFilename\t%s\n\n", g_filename);
} /* get_options */


/* --- main --- */
int main(int argc, char *argv[])
{
	long	startTime;
	int	fd, i;
	byte	*frame, *prev_frame;

	signal(SIGHUP, sighup_handler);
	get_options(argc, argv);

	fd = v4l2_open(videodev, O_RDWR | O_NONBLOCK, 0);
	if (fd < 0) {
		perror(videodev);
		exit(1);
	}

	CLEAR(fmt);
	CLEAR(cap);
	CLEAR(req);

	/* Query capabilities */
	xioctl(fd, VIDIOC_QUERYCAP, &cap);

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "Error: %s not a video capture device.\n",videodev);
		close(fd);
		exit(errno);
	}
	printf("\nCapabilities\n");
	printf("\tdevice:       %s\n",videodev);
	printf("\tdriver:       %s\n",cap.driver);
	printf("\tcard:         %s\n",cap.card);
	printf("\tbus_info:     %s\n",cap.bus_info);
	printf("\tversion:      0x%08X\n",cap.version);
	printf("\tcapabilities: 0x%08X\n",cap.capabilities);
	printf("\t\tVideo Capture:   %d\n", (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) != 0);
	printf("\t\tVideo Output:    %d\n", (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT) != 0);
	printf("\t\tVideo Overlay:   %d\n", (cap.capabilities & V4L2_CAP_VIDEO_OVERLAY) != 0);
	printf("\t\tVBI Capture:     %d\n", (cap.capabilities & V4L2_CAP_VBI_CAPTURE) != 0);
	printf("\t\tVBI Output:      %d\n", (cap.capabilities & V4L2_CAP_VBI_OUTPUT) != 0);
	printf("\t\tSliced VBI Capt: %d\n", (cap.capabilities & V4L2_CAP_SLICED_VBI_CAPTURE) != 0);
	printf("\t\tSliced VBI Out:  %d\n", (cap.capabilities & V4L2_CAP_SLICED_VBI_OUTPUT) != 0);
	printf("\t\tRDS Capture:     %d\n", (cap.capabilities & V4L2_CAP_RDS_CAPTURE) != 0);
	printf("\t\tVIdeo Output Overlay: %d\n", (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT_OVERLAY) != 0);
	printf("\t\tHQ Freq:    %d\n", (cap.capabilities & V4L2_CAP_HW_FREQ_SEEK) != 0);
	printf("\t\tTuner:      %d\n", (cap.capabilities & V4L2_CAP_TUNER) != 0);
	printf("\t\tAudio:      %d\n", (cap.capabilities & V4L2_CAP_AUDIO) != 0);
	printf("\t\tRadio:      %d\n", (cap.capabilities & V4L2_CAP_RADIO) != 0);
	printf("\t\tModulator:  %d\n", (cap.capabilities & V4L2_CAP_MODULATOR) != 0);
	printf("\t\tReadWrite:  %d\n", (cap.capabilities & V4L2_CAP_READWRITE) != 0);
	printf("\t\tAsyncIO:    %d\n", (cap.capabilities & V4L2_CAP_ASYNCIO) != 0);
	printf("\t\tStreaming:  %d\n", (cap.capabilities & V4L2_CAP_STREAMING) != 0);

	/* Read capture format */
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(fd, VIDIOC_G_FMT, &fmt);

	/* Setup capture format */
	printf("\nReport Format:\n");
	printf("\twidth:        %d\n",fmt.fmt.pix.width);
	printf("\theight:       %d\n",fmt.fmt.pix.height);
	printf("\tpixelformat:  %c%c%c%c\n",
				 fmt.fmt.pix.pixelformat&0xFF,
				(fmt.fmt.pix.pixelformat>>8)&0xFF,
				(fmt.fmt.pix.pixelformat>>16)&0xFF,
				(fmt.fmt.pix.pixelformat>>24)&0xFF);
	printf("\tfield:        %d\n",fmt.fmt.pix.field);
	printf("\tbytesperline: %d\n",fmt.fmt.pix.bytesperline);
	printf("\tsizeimage:    %d\n",fmt.fmt.pix.sizeimage);
	printf("\tcolorspace:   %d\n",fmt.fmt.pix.colorspace);

	if (cameraWidth!=0)  fmt.fmt.pix.width       = cameraWidth;
	if (cameraHeight!=0) fmt.fmt.pix.height      = cameraHeight;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

	/* change the foramt */
	xioctl(fd, VIDIOC_S_FMT, &fmt);

	/* read back the new information */
	xioctl(fd, VIDIOC_G_FMT, &fmt);
	cameraWidth        = fmt.fmt.pix.width;
	cameraHeight       = fmt.fmt.pix.height;
	cameraSize         = fmt.fmt.pix.sizeimage;
	cameraBytesperline = fmt.fmt.pix.bytesperline;
	cameraFormat       = fmt.fmt.pix.pixelformat;

	printf("\nCamera Format\n");
	printf("\twidth:        %d\n",cameraWidth);
	printf("\theight:       %d\n",cameraHeight);
	printf("\tpixelformat:  %c%c%c%c\n",
				 cameraFormat&0xFF,
				(cameraFormat>>8)&0xFF,
				(cameraFormat>>16)&0xFF,
				(cameraFormat>>24)&0xFF);
	printf("\tfield:        %d\n",fmt.fmt.pix.field);
	printf("\tbytesperline: %d\n",cameraBytesperline);
	printf("\tsizeimage:    %d\n",cameraSize);
	printf("\tcolorspace:   %d\n",fmt.fmt.pix.colorspace);


	/* send capture request */
	req.count  = 2;
	req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	xioctl(fd, VIDIOC_REQBUFS, &req);

	/* create buffers  */
	buffers = calloc(req.count, sizeof(*buffers));
	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		CLEAR(buf);

		buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory  = V4L2_MEMORY_MMAP;
		buf.index   = n_buffers;

		xioctl(fd, VIDIOC_QUERYBUF, &buf);

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start = v4l2_mmap(NULL, buf.length,
				PROT_READ | PROT_WRITE, MAP_SHARED,
				fd, buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start) {
			perror("mmap");
			exit(EXIT_FAILURE);
		}
        }

	for (i = 0; i < n_buffers; ++i) {
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		xioctl(fd, VIDIOC_QBUF, &buf);
	}

	/* Start streaming */
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(fd, VIDIOC_STREAMON, &type);

	frame      = malloc(cameraWidth * cameraHeight * 3);	// RGB
	prev_frame = malloc(cameraWidth * cameraHeight * 3);	// RGB
	if (!frame || !prev_frame) {
		fprintf(stderr, "Out of memory.\n");
		exit(1);
	}

	/*-----------------------------------------------------------------------
	*  capture one frame
	*-----------------------------------------------------------------------*/
	if (capture(fd, frame)) {
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
	while (timeStamp()-startTime<=totalTime && !stop) {
		/*----------------------------------------------------------
		 *  copy image
		 *----------------------------------------------------------*/
		memcpy(prev_frame, frame, cameraWidth * cameraHeight * 3);
		usleep(time2sleep);

		/*----------------------------------------------------------
		 *  capture next image
		 *----------------------------------------------------------*/
		if (capture(fd, frame)) {
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
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(fd, VIDIOC_STREAMOFF, &type);

	/* close everything */
	for (i = 0; i < n_buffers; ++i)
		v4l2_munmap(buffers[i].start, buffers[i].length);
	v4l2_close(fd);
	exit(0);

	 return 0;
} /* main */
