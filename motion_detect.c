
/*
 * $Id: motion_detect.c,v 1.1 2006/01/10 14:13:32 bnv Exp $
 * $Log: motion_detect.c,v $
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

#define MAX_PORTS 4
#define MAX_RESETS 10

typedef unsigned char	byte;
int DrawText(dc1394_cameracapture *camera, char *txt, int x, int y, int col);

double	threshold    = 200.0;
int	time2sleep   = 5;
int	totalTime    = 86400;

char *g_filename = "image-%08d.ppm";
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
	printf("\tGuid\t%lu\n",g_guid);
	printf("\tThreshold\t%ld\n",threshold);
	printf("\tsleep\t%ld\n",time2sleep);
	printf("\ttime\t%ld\n",time);
	printf("\tfilename\t%s\n",g_filename);
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

/* --- saveImage --- */
int saveImage(dc1394_cameracapture *camera)
{
	FILE	*imagefile;
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
