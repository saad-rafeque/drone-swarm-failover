/* "Swarm Control" launcher: double-click it in the Files app to start the whole ground-control app.
   It runs scripts/start_swarm.sh from the folder the launcher sits in (built by scripts/install_launcher.sh). */
#include <libgen.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int main(void) {
    char self[PATH_MAX], dir[PATH_MAX], script[PATH_MAX + 32];
    ssize_t n = readlink("/proc/self/exe", self, sizeof self - 1);
    if (n < 0) return 1;
    self[n] = '\0';
    strncpy(dir, self, sizeof dir - 1);
    dir[sizeof dir - 1] = '\0';
    snprintf(script, sizeof script, "%s/scripts/start_swarm.sh", dirname(dir));
    execl("/bin/bash", "bash", script, (char *)NULL);
    perror("Swarm Control: cannot run scripts/start_swarm.sh");
    return 1;
}
