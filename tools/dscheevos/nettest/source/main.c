// Wi-Fi smoke test: connect with the firmware's Wi-Fi settings, HTTP GET from the host.
#include <nds.h>
#include <dswifi9.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
static void out(const char* s) { char b[300] = "@@"; strncat(b, s, 290); strcat(b, "\n"); nocashMessage(b); iprintf("%s", b + 2); }
int main(void) {
    char line[300];
    consoleDemoInit();
    out("START nettest");
    if (!Wifi_InitDefault(WFC_CONNECT)) { out("FAIL wifi connect"); goto end; }
    struct in_addr ip = Wifi_GetIPInfo(0, 0, 0, 0);
    siprintf(line, "WIFI connected ip=%s", inet_ntoa(ip)); out(line);
    int s = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in sa = {0};
    sa.sin_family = AF_INET; sa.sin_port = htons(8080); sa.sin_addr.s_addr = inet_addr("10.64.0.1");
    if (connect(s, (struct sockaddr*)&sa, sizeof sa) != 0) { out("FAIL tcp connect"); goto end; }
    const char* req = "GET /hello.txt HTTP/1.0\r\nHost: 10.64.0.1\r\n\r\n";
    send(s, req, strlen(req), 0);
    char buf[512]; int n, total = 0; char body[512] = "";
    while ((n = recv(s, buf, sizeof buf - 1, 0)) > 0) { buf[n] = 0; if (total + n < 500) strcat(body, buf); total += n; }
    closesocket(s);
    char* b = strstr(body, "\r\n\r\n");
    siprintf(line, "HTTP got %d bytes, body: %s", total, b ? b + 4 : "(none)"); out(line);
end:
    out("DONE");
    while (1) swiWaitForVBlank();
}
