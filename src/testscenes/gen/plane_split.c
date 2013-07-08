#include <stdio.h>
#include <stdlib.h>

double max(double a, double b) {
    return a > b ? a : b;
}

typedef enum bool_enum {
    FALSE, TRUE
} bool;

typedef struct point_struct {
    double x, y, z;
} point;

typedef struct vector {
    double x, y, z;
} vector;

bool read_point(char *str, point *pt) {
    return sscanf(str, "%lf,%lf,%lf", &pt->x, &pt->y, &pt->z) == 3;
}

void minus(point a, point b, vector *v) {
    v->x = a.x - b.x;
    v->y = a.y - b.y;
    v->z = a.z - b.z;
}

void divide(vector *v, double d) {
    v->x /= d;
    v->y /= d;
    v->z /= d;
}

int main(int argc, char **argv) {
    if (argc > 5 || argc < 4) {
        fprintf(stderr, "Usage: %s <size>\n", argv[0]);
        return 2;
    }
    
    point p1, p2, p3;
    
    if (!read_point(argv[1], &p1)) {
        fprintf(stderr, "Invalid point: %s\n", argv[1]);
        return 1;
    }
    if (!read_point(argv[2], &p2)) {
        fprintf(stderr, "Invalid point: %s\n", argv[2]);
        return 1;
    }
    if (!read_point(argv[3], &p3)) {
        fprintf(stderr, "Invalid point: %s\n", argv[3]);
        return 1;
    }
    
    int size = 100;
    if (argc == 5)
        size = atoi(argv[4]);
    if (size < 0) {
        fprintf(stderr, "Please enter a positive size: %d\n", size);
    }
    
    vector dir1, dir2;
    
    minus(p2, p1, &dir1);
    minus(p3, p1, &dir2);
    
    int count;
    {
        double count_x, count_y, count_z;
        
        count_x = max(dir1.x / size, dir2.x / size);
        count_y = max(dir1.y / size, dir2.y / size);
        count_z = max(dir1.z / size, dir2.z / size);
        
        count = (int) max(count_x, max(count_y, count_z));
    }
    
    divide(&dir1, count);
    divide(&dir2, count);
    
    int x, y;
    int idx0, idx1, idx2, idx3;
    
    printf("Shape \"trianglemesh\"\n");
    printf("\t\"point P\" [\n");
    for (x = 0; x <= count; x++) {
        for (y = 0; y <= count; y++) {
            printf("\t\t%f %f %f\n",
                p1.x + (x * dir1.x) + (y * dir2.x),
                p1.y + (x * dir1.y) + (y * dir2.y),
                p1.z + (x * dir1.z) + (y * dir2.z)
            );
        }
    }
    printf("\t\n]");
    
    printf("\t\"integer indices\" [\n");
    for (x = 0; x < count; x++) {
        for (y = 0; y < count; y++) {
            idx0 = x       * (count + 1) + y;
            idx1 = (x + 1) * (count + 1) + y;
            idx2 = (x + 1) * (count + 1) + (y + 1);
            idx3 = x       * (count + 1) + (y + 1);
            
            printf("\t\t%d %d %d %d %d %d\n",
                idx0, idx1, idx2,
                idx2, idx3, idx0
            );
        }
    }
    printf("\t]\n");
    
    return 0;
}
