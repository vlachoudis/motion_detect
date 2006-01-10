
/*
 * $Id: motion_detect.c,v 1.2 2006/01/10 15:24:32 bnv Exp $
 * $Log: motion_detect.c,v $
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

#include <stdio.h>
#include <stdlib.h>

#include <stdio.h>
#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#define _GNU_SOURCE
#include <getopt.h>
#include <time.h>
#include <sys/time.h>
#include <sys/unistd.h>
#include <stdio.h>
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

typedef unsigned char	byte;
int DrawText(dc1394_cameracapture *camera, char *txt, int x, int y, int col);

double	threshold    = 200.0;
int	time2sleep   = 5;
int	totalTime    = 86400;

char *g_filename = "image-%08d.jpg";
u_int64_t g_guid = 0;

static struct option long_options[] = {
	{"guid", 1, NULL, 0},
	{"threshold",1,NULL, 0},
	{"sleep",1,NULL, 0},
	{"time",1,NULL, 0},
	{"help", 0, NULL, 0},
	{NULL, 0, 0, 0}
};

/* --- get_options --- */
void get_options(int argc, char *argv[])
{
	int option_index = 0;

	while (getopt_long(argc, argv, "", long_options, &option_index) >= 0) {
		switch (option_index) {
			/* case values must match long_options */
			case 0:
				sscanf(optarg, "%llx", &g_guid);
				break;
			case 1:
				sscanf(optarg, "%lg", &threshold);
				break;
			case 2:
				sscanf(optarg, "%d", &time2sleep);
				break;
			case 3:
				sscanf(optarg, "%d", &totalTime);
				break;
			default:
				printf("\n"
				       "%s - grab a color sequence using format0, rgb mode\n\n"
				       "Usage:\n"
				       "    %s [--guid=/dev/video1394/x] [filename.ppm]\n\n"
				       "    --threshold - specifies RMS threshold to use (default: 200)\n"
				       "    --sleep     - specifies seconds to sleep (default: 5)\n"
				       "    --time      - specifies seconds to run (default: 86400)\n"
				       "    --guid      - specifies camera to use (optional)\n"
				       "                  default = first identified on buses\n"
				       "    --help      - prints this message\n"
				       "    filename is optional; the default is image%%08d.ppm\n\n",
				       argv[0], argv[0]);
				exit(0);
		}
	}
	if (optind < argc)
		g_filename = argv[optind];

	printf("Options\n");
	printf("\tGuid\t\t%lu\n",(unsigned long)g_guid);
	printf("\tThreshold\t%g\n",threshold);
	printf("\tSleep Time\t%d s\n",time2sleep);
	printf("\tFilename\t%s\n\n",g_filename);
} /* get_options */

/* --- compareFrames --- */
int compareFrames(dc1394_cameracapture *camera, byte *prev_buffer)
{
	int	x,y;
	double	sum2 = 0.0;
	double	rms;
	byte	*curr, *prev;

	curr = (byte*)camera->capture_buffer;
	prev = (byte*)prev_buffer;

	for (y=0; y<camera->frame_height; y++)
		for (x=0; x<camera->frame_width; x++) {
			double	currGray, prevGray;
			int	red, green, blue;
			int	diff;

			red   = *curr++;
			green = *curr++;
			blue  = *curr++;
			currGray = 0.3  * (double)red +
				   0.59 * (double)green +
				   0.11 * (double)blue;

			red   = *prev++;
			green = *prev++;
			blue  = *prev++;
			prevGray = 0.3  * (double)red +
				   0.59 * (double)green +
				   0.11 * (double)blue;

			diff = currGray-prevGray;
			sum2 += diff*diff;
		}

	rms = sum2/(double)(camera->frame_height*camera->frame_width);
	printf("SUM2=%lg RMS=%lg\n",sum2,rms);

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
GLOBAL(void) writeJPEGFile(char *filename, int quality, dc1394_cameracapture *camera)
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
	cinfo.image_width = camera->frame_width;
	cinfo.image_height = camera->frame_height;
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
	row_stride = camera->frame_width * 3;	/* JSAMPLEs per row in image_buffer */

	while (cinfo.next_scanline < cinfo.image_height) {
		/* jpeg_write_scanlines expects an array of pointers to scanlines.
		 * Here the array is only one element long, but you could pass
		 * more than one scanline at a time if that's more convenient.
		 */
		row_pointer[0] = & ((JSAMPLE*)camera->capture_buffer)[cinfo.next_scanline * row_stride];
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

/* --- writePPMFile --- */
int writePPMFile(char *filename, dc1394_cameracapture *camera)
{
	FILE	*imagefile;

	imagefile = fopen(filename, "w");
	if (imagefile == NULL)
		return 1;

	fprintf(imagefile, "P6\n%u %u\n255\n", camera->frame_width,
		camera->frame_height);
	fwrite((const char *)camera->capture_buffer, 1,
	       camera->frame_height * camera->frame_width * 3, imagefile);
	fclose(imagefile);
	printf("wrote: %s\n", filename);
	return 0;
} /* writePPMFile */

/* --- saveImage --- */
int saveImage(dc1394_cameracapture *camera)
{
	char	filename[128];
	char	str[256];
	time_t	now;
	struct tm *tmdata ;
	int	xt,yt;

	now = time(NULL);
	tmdata = localtime(&now) ;
	sprintf(str,"Vasilis Office %d/%d/%d %d:%02d:%02d",
			tmdata->tm_mday,
			tmdata->tm_mon+1,
			tmdata->tm_year+1900,
			tmdata->tm_hour,
			tmdata->tm_min,
			tmdata->tm_sec);

	xt = 5;
	yt = camera->frame_height-24;
	DrawText(camera,str,xt,yt,0x1F1F1F);
	DrawText(camera,str,xt-2,yt-2,0xFFFF7F);

	sprintf(filename,g_filename,timeStamp());
	//writePPMFile(filename,camera);
	writeJPEGFile(filename,80,camera);
	return 0;
} /* saveImage */

/* --- main --- */
int main(int argc, char *argv[])
{
	dc1394_cameracapture camera;
	byte	*prev_buffer;
	struct	raw1394_portinfo ports[MAX_PORTS];
	int	numPorts = 0;
	int	numCameras = 0;
	int	i, j;
	int	found = 0;
	long	startTime;
	raw1394handle_t handle;
	nodeid_t *camera_nodes = NULL;

	get_options(argc, argv);

	/*-----------------------------------------------------------------------
	 *  Open ohci and asign handle to it
	 *-----------------------------------------------------------------------*/
	handle = raw1394_new_handle();
	if (handle == NULL) {
		fprintf(stderr, "Unable to aquire a raw1394 handle\n\n"
			"Please check \n"
			"  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
			"  - if you have read/write access to /dev/raw1394\n\n");
		exit(1);
	}
	/* get the number of ports (cards) */
	numPorts = raw1394_get_port_info(handle, ports, numPorts);
	raw1394_destroy_handle(handle);
	handle = NULL;

	for (j = 0; j < MAX_RESETS && found == 0; j++) {
		/* look across all ports for cameras */
		for (i = 0; i < numPorts && found == 0; i++) {
			if (handle != NULL)
				dc1394_destroy_handle(handle);
			handle = dc1394_create_handle(i);
			if (handle == NULL) {
				fprintf(stderr,
					"Unable to aquire a raw1394 handle for port %i\n",
					i);
				exit(1);
			}
			numCameras = 0;
			camera_nodes = dc1394_get_camera_nodes(handle,
							       &numCameras, 0);
			if (numCameras > 0) {
				if (g_guid == 0) {
					dc1394_camerainfo info;

					/* use the first camera found */
					camera.node = camera_nodes[0];
					if (dc1394_get_camera_info
					    (handle, camera_nodes[0],
					     &info) == DC1394_SUCCESS)
						dc1394_print_camera_info(&info);
					found = 1;
				} else {
					/* attempt to locate camera by guid */
					int k;

					for (k = 0;
					     k < numCameras && found == 0;
					     k++) {
						dc1394_camerainfo info;

						if (dc1394_get_camera_info
						    (handle, camera_nodes[k],
						     &info) == DC1394_SUCCESS) {
							if (info.euid_64 ==
							    g_guid) {
								dc1394_print_camera_info
										(&info);
								camera.node = camera_nodes[k];
								found = 1;
							}
						}
					}
				}
				if (found == 1) {
					/* camera can not be root--highest order node */
					if (camera.node ==
					    raw1394_get_nodecount(handle) - 1) {
						/* reset and retry if root */
						raw1394_reset_bus(handle);
						sleep(2);
						found = 0;
					}
				}
				dc1394_free_camera_nodes(camera_nodes);
			}	/* cameras >0 */
		}		/* next port */
	}			/* next reset retry */

	if (found == 0 && g_guid != 0) {
		fprintf(stderr, "Unable to locate camera node by guid\n");
		exit(1);
	} else if (numCameras == 0) {
		fprintf(stderr, "no cameras found :(\n");
		dc1394_destroy_handle(handle);
		exit(1);
	}
	if (j == MAX_RESETS) {
		fprintf(stderr, "failed to not make camera root node :(\n");
		dc1394_destroy_handle(handle);
		exit(1);
	}

	/*-----------------------------------------------------------------------
	 *  setup capture
	 *-----------------------------------------------------------------------*/
	if (dc1394_setup_capture(handle, camera.node, 0,	/* channel */
				 FORMAT_VGA_NONCOMPRESSED,
				 MODE_640x480_RGB,
				 SPEED_400,
				 FRAMERATE_7_5, &camera) != DC1394_SUCCESS) {
		fprintf(stderr, "unable to setup camera-\n"
			"check line %d of %s to make sure\n"
			"that the video mode,framerate and format are\n"
			"supported by your camera\n", __LINE__, __FILE__);
		dc1394_release_camera(handle, &camera);
		dc1394_destroy_handle(handle);
		exit(1);
	}

	/*-----------------------------------------------------------------------
	 *  have the camera start sending us data
	 *-----------------------------------------------------------------------*/
	if (dc1394_start_iso_transmission(handle, camera.node)
	    != DC1394_SUCCESS) {
		fprintf(stderr, "unable to start camera iso transmission\n");
		dc1394_release_camera(handle, &camera);
		dc1394_destroy_handle(handle);
		exit(1);
	}

	/*-------------------------------------------------------------------
	 *  capture first frame
	 *-------------------------------------------------------------------*/
	if (dc1394_single_capture(handle, &camera) != DC1394_SUCCESS) {
		fprintf(stderr, "unable to capture a frame\n");
		dc1394_release_camera(handle, &camera);
		dc1394_destroy_handle(handle);
		exit(1);
	}

	if (saveImage(&camera)) {
		perror("Can't create output file");
		dc1394_release_camera(handle, &camera);
		dc1394_destroy_handle(handle);
		exit(1);
	}

	prev_buffer = (byte *)malloc(camera.frame_width * camera.frame_height * 3);

	startTime = timeStamp();
	while (timeStamp()-startTime<=totalTime) {
		/*----------------------------------------------------------
		 *  copy image
		 *----------------------------------------------------------*/
		memcpy(prev_buffer, camera.capture_buffer,
		       camera.frame_width * camera.frame_height * 3);

		sleep(time2sleep);

		/*----------------------------------------------------------
		 *  capture next image
		 *----------------------------------------------------------*/
		if (dc1394_single_capture(handle, &camera) != DC1394_SUCCESS) {
			fprintf(stderr, "unable to capture a frame\n");
			dc1394_release_camera(handle, &camera);
			dc1394_destroy_handle(handle);
			exit(1);
		}

		/*---------------------------------------------------------
		 *  compare and save
		 *---------------------------------------------------------*/
		if (compareFrames(&camera, prev_buffer))
			if (saveImage(&camera)) {
				perror("Can't create output file");
				dc1394_release_camera(handle, &camera);
				dc1394_destroy_handle(handle);
				exit(1);
			}
	}

	/*-----------------------------------------------------------------------
	 *  Close camera
	 *-----------------------------------------------------------------------*/
	dc1394_release_camera(handle, &camera);
	dc1394_destroy_handle(handle);
	return 0;
} /* main */
