#ifndef MAIN_H
#define MAIN_H

/* Logformat fieldtype 4 row to the csv log and every tcp client.
   status: 1 started, 2 aborted, 3 done. */
void workorder_status(int status, const char *name);

#endif
