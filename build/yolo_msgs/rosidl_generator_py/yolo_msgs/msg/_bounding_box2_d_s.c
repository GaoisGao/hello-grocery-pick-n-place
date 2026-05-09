// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from yolo_msgs:msg/BoundingBox2D.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "yolo_msgs/msg/detail/bounding_box2_d__struct.h"
#include "yolo_msgs/msg/detail/bounding_box2_d__functions.h"

bool yolo_msgs__msg__pose2_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * yolo_msgs__msg__pose2_d__convert_to_py(void * raw_ros_message);
bool yolo_msgs__msg__vector2__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * yolo_msgs__msg__vector2__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool yolo_msgs__msg__bounding_box2_d__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
    if (class_attr == NULL) {
      return false;
    }
    PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
    if (name_attr == NULL) {
      Py_DECREF(class_attr);
      return false;
    }
    PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
    if (module_attr == NULL) {
      Py_DECREF(name_attr);
      Py_DECREF(class_attr);
      return false;
    }

    // PyUnicode_1BYTE_DATA is just a cast
    assert(strncmp("yolo_msgs.msg._bounding_box2_d", (char *)PyUnicode_1BYTE_DATA(module_attr), 30) == 0);
    assert(strncmp("BoundingBox2D", (char *)PyUnicode_1BYTE_DATA(name_attr), 13) == 0);

    Py_DECREF(module_attr);
    Py_DECREF(name_attr);
    Py_DECREF(class_attr);
  }
  yolo_msgs__msg__BoundingBox2D * ros_message = _ros_message;
  {  // center
    PyObject * field = PyObject_GetAttrString(_pymsg, "center");
    if (!field) {
      return false;
    }
    if (!yolo_msgs__msg__pose2_d__convert_from_py(field, &ros_message->center)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // size
    PyObject * field = PyObject_GetAttrString(_pymsg, "size");
    if (!field) {
      return false;
    }
    if (!yolo_msgs__msg__vector2__convert_from_py(field, &ros_message->size)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * yolo_msgs__msg__bounding_box2_d__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BoundingBox2D */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("yolo_msgs.msg._bounding_box2_d");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BoundingBox2D");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  yolo_msgs__msg__BoundingBox2D * ros_message = (yolo_msgs__msg__BoundingBox2D *)raw_ros_message;
  {  // center
    PyObject * field = NULL;
    field = yolo_msgs__msg__pose2_d__convert_to_py(&ros_message->center);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "center", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // size
    PyObject * field = NULL;
    field = yolo_msgs__msg__vector2__convert_to_py(&ros_message->size);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "size", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
