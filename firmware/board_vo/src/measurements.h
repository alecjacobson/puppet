#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#define MAX_NUM_MEASUREMENTS 3

typedef struct {
    int i2c_request;
    unsigned int num_measurements;
    char buffer[MAX_NUM_MEASUREMENTS * 2];
    unsigned short angle[MAX_NUM_MEASUREMENTS];
} Measurements;

extern Measurements measurements;

void initialize_measurements();

void collect_joint_measurements(int num_sensors);

void pack_joint_measurements_for_i2c();

void create_joint_message(int node_id, Line *l);



#endif // MEASUREMENTS_H

