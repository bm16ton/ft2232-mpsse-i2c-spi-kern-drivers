#include <stdio.h>
#include <libserialport.h>
#include <jni.h>
#include <string.h>

JNIEXPORT jstring JNICALL Java_processing_app_Platform_resolveDeviceAttachedToNative
  (JNIEnv * env, jobject jobj, jstring serial)
{
	struct sp_port *port;

	char vid_pid_iserial[256] = " ";
	const char *portname = (*env)->GetStringUTFChars(env, serial, NULL);
	jstring result;

	if (sp_get_port_by_name(portname, &port) != SP_OK) {
		return (*env)->NewStringUTF(env, "");
	}

	int vid, pid;
	if (sp_get_port_usb_vid_pid(port, &vid, &pid) == SP_OK) {
		snprintf(vid_pid_iserial, sizeof(vid_pid_iserial), "0x%04X_0x%04X_%s_%s", vid, pid, sp_get_port_usb_serial(port), sp_get_port_description(port));
	}

	sp_free_port(port);
	(*env)->ReleaseStringUTFChars(env, serial, portname);

	return (*env)->NewStringUTF(env, vid_pid_iserial);
}

JNIEXPORT jobjectArray JNICALL Java_processing_app_Platform_listSerialsNative
  (JNIEnv * env, jobject jobj)
{
	struct sp_port **ports;
	jobjectArray ret;
	int i;

	char portname_vid_pid[256] = " ";

	if (sp_list_ports(&ports) != SP_OK) {
		return (jobjectArray)(*env)->NewObjectArray(env, 0, (*env)->FindClass(env, "java/lang/String"), (*env)->NewStringUTF(env, ""));
	}

	// like ports.size()
	for (i = 0; ports[i]; i++) {};

	ret = (jobjectArray)(*env)->NewObjectArray(env, i, (*env)->FindClass(env, "java/lang/String"), (*env)->NewStringUTF(env, ""));

	int vid, pid;
	for (i = 0; ports[i]; i++) {
		int vid, pid;
		if (sp_get_port_usb_vid_pid(ports[i], &vid, &pid) == SP_OK) {
			snprintf(portname_vid_pid, sizeof(portname_vid_pid), "%s_%04X_%04X", sp_get_port_name(ports[i]), vid, pid);
		} else {
			snprintf(portname_vid_pid, sizeof(portname_vid_pid), "%s_%04X_%04X", sp_get_port_name(ports[i]), 0, 0);
		}
		(*env)->SetObjectArrayElement(env, ret, i, (*env)->NewStringUTF(env, portname_vid_pid));
	}

	sp_free_port_list(ports);

	return ret;
}
