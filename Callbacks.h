#include <AntTweakBar.h>

// No-op setter, does nothing
void TW_CALL no_op(const void *value, void *clientData);
// No-op getter, does nothing
void TW_CALL no_op(void *value, void *clientData);
// Function called by AntTweakBar to copy the content of a std::string handled
// by the AntTweakBar library to a std::string handled by your application
void TW_CALL CopyStdStringToClient(std::string& destinationClientString, const std::string& sourceLibraryString);
// Bind nodes to current arrangment
void TW_CALL bindCB(void *clientData);
void TW_CALL rigCB(void *clientData);
// Snap camera to canonical view
void TW_CALL snapCB(void *clientData);
void TW_CALL xyCB(void *clientData);
void TW_CALL xzCB(void *clientData);
void TW_CALL yzCB(void *clientData);
void TW_CALL orthoCB(void *clientData);
// Get mesh sizes
void TW_CALL get_mesh_VrowsCB(void * value, void *clientData);
void TW_CALL get_mesh_FrowsCB(void * value, void *clientData);
// Set/gett cpu_lbs
void TW_CALL set_cpu_lbsCB(const void * value, void *clientData);
void TW_CALL get_cpu_lbsCB(void * value, void *clientData);
void TW_CALL set_auto_fit_activatedCB(const void * value, void *clientData);
void TW_CALL get_auto_fit_activatedCB(void * value, void *clientData);
// Reset all node offsets
void TW_CALL reset_all_offsetsCB(void *clientData);
void TW_CALL reset_all_anglesCB(void *clientData);
void TW_CALL reset_all_drag_arrowsCB(void *clientData);
void TW_CALL living_willCB(void *clientData);
void TW_CALL set_mesh_invert_orientationCB(const void * value, void *clientData);
void TW_CALL get_mesh_invert_orientationCB(void * value, void *clientData);
void TW_CALL set_rotation_typeCB(const void * value, void *clientData);
void TW_CALL get_rotation_typeCB(void * value, void *clientData);
void TW_CALL reset_rotationsCB(void *clientData);
