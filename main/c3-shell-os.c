#include <stdio.h>
#include <stdbool.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "lwip/netdb.h"
#include "config.h"

static char sys_ip[16] = "0.0.0.0";
static i2c_master_dev_handle_t oled_handle;
static bool wifi_connected;
static bool telnet_enabled;
static SemaphoreHandle_t shell_mutex;

typedef enum {
    WIFI_STATE_IDLE = 0,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_FAILED,
} wifi_state_t;

static wifi_state_t wifi_state = WIFI_STATE_IDLE;
static int wifi_retry_count;

typedef int (*shell_command_fn_t)(int argc, char **argv);
typedef void (*shell_writer_fn_t)(void *ctx, const char *text, size_t len);

typedef struct {
    const char *name;
    const char *help;
    shell_command_fn_t fn;
} shell_command_t;

static shell_writer_fn_t active_shell_writer;
static void *active_shell_writer_ctx;

static void oled_send_cmd(uint8_t cmd);

#define WIFI_MAX_RETRIES        10
#define HTTP_BUFFER_SIZE        512
#define SHELL_PATH_SIZE         256
#define PY_STRING_ARG_SIZE      160
#define SHELL_MAX_ARGS          12
#define SHELL_LINE_SIZE         192
#define TELNET_PORT             23
#define TELNET_BACKLOG          1
#define TELNET_IO_TIMEOUT_MS    1000

#define OLED_WIDTH              72
#define OLED_HEIGHT             40
#define OLED_PAGES              (OLED_HEIGHT / 8)
#define OLED_COLUMN_OFFSET      28
#define OLED_LINE_CHARS         (OLED_WIDTH / 6)

static const uint8_t oled_font[41][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x7A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43},
    {0x3E, 0x45, 0x49, 0x51, 0x3E},
    {0x00, 0x21, 0x7F, 0x01, 0x00},
    {0x21, 0x43, 0x45, 0x49, 0x31},
    {0x42, 0x41, 0x51, 0x69, 0x46},
    {0x0C, 0x14, 0x24, 0x7F, 0x04},
    {0x72, 0x51, 0x51, 0x51, 0x4E},
    {0x1E, 0x29, 0x49, 0x49, 0x06},
    {0x40, 0x47, 0x48, 0x50, 0x60},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x30, 0x49, 0x49, 0x4A, 0x3C},
    {0x00, 0x36, 0x36, 0x00, 0x00},
    {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x40, 0x30, 0x08, 0x06, 0x01}
};

static uint8_t oled_framebuffer[OLED_WIDTH * OLED_PAGES];

static int do_help(int argc, char **argv);
static int do_df(int argc, char **argv);
static int do_pwd(int argc, char **argv);
static int do_ls(int argc, char **argv);
static int do_cd(int argc, char **argv);
static int do_mkdir(int argc, char **argv);
static int do_touch(int argc, char **argv);
static int do_cat(int argc, char **argv);
static int do_write(int argc, char **argv);
static int do_append(int argc, char **argv);
static int do_ip(int argc, char **argv);
static int do_rm(int argc, char **argv);
static int do_wifi(int argc, char **argv);
static int do_free(int argc, char **argv);
static int do_uname(int argc, char **argv);
static int do_echo(int argc, char **argv);
static int do_ping(int argc, char **argv);
static int do_wget(int argc, char **argv);
static int do_python(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    {"help", "List commands", do_help},
    {"df", "Disk usage", do_df},
    {"pwd", "Print current working directory", do_pwd},
    {"ls", "List files", do_ls},
    {"cd", "Change directory", do_cd},
    {"mkdir", "Create directory", do_mkdir},
    {"touch", "Create empty file", do_touch},
    {"cat", "Print file contents", do_cat},
    {"write", "Write text to file", do_write},
    {"append", "Append text to file", do_append},
    {"rm", "Remove file or empty directory", do_rm},
    {"ip", "Show network IP/port", do_ip},
    {"wifi", "Show Wi-Fi configuration/status", do_wifi},
    {"free", "Show free heap memory", do_free},
    {"uname", "Show system info", do_uname},
    {"echo", "Print arguments", do_echo},
    {"ping", "Ping a host", do_ping},
    {"wget", "Download a file", do_wget},
    {"py", "Alias for python command", do_python},
    {"python", "Run tiny Python-style scripts", do_python},
};

static const size_t shell_command_count = sizeof(shell_commands) / sizeof(shell_commands[0]);

static void shell_stdout_writer(void *ctx, const char *text, size_t len)
{
    (void)ctx;
    fwrite(text, 1, len, stdout);
    fflush(stdout);
}

static void shell_socket_writer(void *ctx, const char *text, size_t len)
{
    int sock = (int)(intptr_t)ctx;
    size_t sent = 0;

    while (sent < len) {
        int rc = send(sock, text + sent, len - sent, 0);

        if (rc <= 0) {
            break;
        }
        sent += (size_t)rc;
    }
}

static void shell_write_raw(const char *text, size_t len)
{
    shell_writer_fn_t writer = active_shell_writer != NULL ? active_shell_writer : shell_stdout_writer;
    void *ctx = active_shell_writer_ctx;

    writer(ctx, text, len);
}

static void shell_printf(const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    int written;

    va_start(args, fmt);
    written = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (written <= 0) {
        return;
    }
    if (written > (int)sizeof(buffer)) {
        written = sizeof(buffer);
    }

    shell_write_raw(buffer, (size_t)written);
}

static const char *wifi_status_text(void)
{
    switch (wifi_state) {
    case WIFI_STATE_CONNECTING:
        return "WIFI JOIN";
    case WIFI_STATE_CONNECTED:
        return "WIFI READY";
    case WIFI_STATE_FAILED:
        return "WIFI FAIL";
    default:
        return "WIFI IDLE";
    }
}

static const char *shell_display_path(void)
{
    static char display_path[SHELL_PATH_SIZE];
    static const char *prefix = "/spiffs";
    char cwd[SHELL_PATH_SIZE];

    if (getcwd(cwd, sizeof(cwd)) == NULL) {
        return "/";
    }

    if (strncmp(cwd, prefix, strlen(prefix)) == 0) {
        const char *suffix = cwd + strlen(prefix);

        if (*suffix == '\0') {
            return "/";
        }
        snprintf(display_path, sizeof(display_path), "%s", suffix);
        return display_path;
    }

    snprintf(display_path, sizeof(display_path), "%s", cwd);
    return display_path;
}

static void print_prompt(void)
{
    shell_printf("assix@c3-os:%s$ ", shell_display_path());
}

static const shell_command_t *shell_find_command(const char *name)
{
    size_t index;

    for (index = 0; index < shell_command_count; ++index) {
        if (strcmp(shell_commands[index].name, name) == 0) {
            return &shell_commands[index];
        }
    }

    return NULL;
}

static int shell_execute_line(const char *line, shell_writer_fn_t writer, void *writer_ctx)
{
    char line_copy[SHELL_LINE_SIZE];
    char *argv[SHELL_MAX_ARGS];
    const shell_command_t *command;
    shell_writer_fn_t previous_writer;
    void *previous_writer_ctx;
    int argc;
    int rc;

    if (line == NULL) {
        return 1;
    }

    snprintf(line_copy, sizeof(line_copy), "%s", line);
    argc = esp_console_split_argv(line_copy, argv, SHELL_MAX_ARGS);
    if (argc <= 0) {
        return 0;
    }

    command = shell_find_command(argv[0]);
    if (command == NULL) {
        previous_writer = active_shell_writer;
        previous_writer_ctx = active_shell_writer_ctx;
        active_shell_writer = writer;
        active_shell_writer_ctx = writer_ctx;
        shell_printf("Command not found\n");
        active_shell_writer = previous_writer;
        active_shell_writer_ctx = previous_writer_ctx;
        return 1;
    }

    xSemaphoreTake(shell_mutex, portMAX_DELAY);
    previous_writer = active_shell_writer;
    previous_writer_ctx = active_shell_writer_ctx;
    active_shell_writer = writer;
    active_shell_writer_ctx = writer_ctx;
    rc = command->fn(argc, argv);
    active_shell_writer = previous_writer;
    active_shell_writer_ctx = previous_writer_ctx;
    xSemaphoreGive(shell_mutex);

    return rc;
}

static bool telnet_send_text(int sock, const char *text)
{
    size_t len = strlen(text);
    size_t sent = 0;

    while (sent < len) {
        int rc = send(sock, text + sent, len - sent, 0);

        if (rc <= 0) {
            return false;
        }
        sent += (size_t)rc;
    }

    return true;
}

static bool telnet_read_line(int sock, char *buffer, size_t buffer_size, bool echo)
{
    size_t pos = 0;

    while (true) {
        uint8_t ch = 0;
        int rc = recv(sock, &ch, 1, 0);

        if (rc <= 0) {
            return false;
        }
        if (ch == 0xFF) {
            uint8_t discard[2];

            recv(sock, discard, sizeof(discard), 0);
            continue;
        }
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            buffer[pos] = '\0';
            if (!telnet_send_text(sock, "\r\n")) {
                return false;
            }
            return true;
        }
        if (ch == '\b' || ch == 0x7F) {
            if (pos > 0) {
                pos--;
                if (echo && !telnet_send_text(sock, "\b \b")) {
                    return false;
                }
            }
            continue;
        }
        if (isprint(ch) && pos + 1 < buffer_size) {
            buffer[pos++] = (char)ch;
            if (echo) {
                char out[2] = {(char)ch, '\0'};

                if (!telnet_send_text(sock, out)) {
                    return false;
                }
            }
        }
    }
}

static void telnet_handle_client(int client_sock)
{
    char username[64];
    char password[64];
    char line[SHELL_LINE_SIZE];

    chdir("/spiffs");
    telnet_send_text(client_sock, "\r\nassix-c3-os telnet\r\n");
    telnet_send_text(client_sock, "login: ");
    if (!telnet_read_line(client_sock, username, sizeof(username), true)) {
        return;
    }
    telnet_send_text(client_sock, "password: ");
    if (!telnet_read_line(client_sock, password, sizeof(password), false)) {
        return;
    }

    if (strcmp(username, SHELL_USER) != 0 || strcmp(password, SHELL_PASS) != 0) {
        telnet_send_text(client_sock, "Authentication failed\r\n");
        return;
    }

    telnet_send_text(client_sock, "Authenticated\r\n");
    while (true) {
        telnet_send_text(client_sock, "assix@c3-os:");
        telnet_send_text(client_sock, shell_display_path());
        telnet_send_text(client_sock, "$ ");
        if (!telnet_read_line(client_sock, line, sizeof(line), true)) {
            return;
        }
        if (strcmp(line, "exit") == 0 || strcmp(line, "logout") == 0) {
            telnet_send_text(client_sock, "Bye\r\n");
            return;
        }
        shell_execute_line(line, shell_socket_writer, (void *)(intptr_t)client_sock);
    }
}

static void telnet_server_task(void *pvParameters)
{
    int listen_sock = -1;
    struct timeval timeout = {
        .tv_sec = 1,
        .tv_usec = 0,
    };

    (void)pvParameters;
    chdir("/spiffs");
    while (true) {
        if (!wifi_connected) {
            if (listen_sock >= 0) {
                close(listen_sock);
                listen_sock = -1;
            }
            telnet_enabled = false;
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        if (listen_sock < 0) {
            struct sockaddr_in server_addr = {
                .sin_family = AF_INET,
                .sin_port = htons(TELNET_PORT),
                .sin_addr.s_addr = htonl(INADDR_ANY),
            };
            int opt = 1;

            listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
            if (listen_sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0 ||
                listen(listen_sock, TELNET_BACKLOG) != 0) {
                close(listen_sock);
                listen_sock = -1;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            telnet_enabled = true;
            printf("[TELNET] Listening on port %d\n", TELNET_PORT);
        }

        {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_len);

            if (client_sock < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                close(listen_sock);
                listen_sock = -1;
                telnet_enabled = false;
                continue;
            }

            printf("[TELNET] Client connected\n");
            telnet_handle_client(client_sock);
            close(client_sock);
            printf("[TELNET] Client disconnected\n");
        }
    }
}

static esp_err_t shell_resolve_path(const char *path, char *resolved, size_t resolved_size)
{
    char cwd[SHELL_PATH_SIZE];

    if (path == NULL || resolved == NULL || resolved_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (path[0] == '\0' || strcmp(path, ".") == 0) {
        if (getcwd(resolved, resolved_size) == NULL || resolved[0] == '\0' || strcmp(resolved, "/") == 0) {
            strlcpy(resolved, "/spiffs", resolved_size);
        }
        return ESP_OK;
    }

    if (strncmp(path, "/spiffs", 7) == 0) {
        if (snprintf(resolved, resolved_size, "%s", path) >= (int)resolved_size) {
            return ESP_ERR_INVALID_SIZE;
        }
        return ESP_OK;
    }

    if (path[0] == '/') {
        if (snprintf(resolved, resolved_size, "/spiffs%s", path) >= (int)resolved_size) {
            return ESP_ERR_INVALID_SIZE;
        }
        return ESP_OK;
    }

    if (getcwd(cwd, sizeof(cwd)) == NULL || cwd[0] == '\0' || strcmp(cwd, "/") == 0) {
        strlcpy(cwd, "/spiffs", sizeof(cwd));
    }
    if (snprintf(resolved, resolved_size, "%s/%s", cwd, path) >= (int)resolved_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

static const char *shell_pretty_path(const char *resolved)
{
    static char pretty_path[SHELL_PATH_SIZE];
    static const char *prefix = "/spiffs";

    if (resolved == NULL) {
        return "?";
    }
    if (strncmp(resolved, prefix, strlen(prefix)) == 0) {
        const char *suffix = resolved + strlen(prefix);

        if (*suffix == '\0') {
            return "/";
        }
        snprintf(pretty_path, sizeof(pretty_path), "%s", suffix);
        return pretty_path;
    }

    return resolved;
}

static int wget_download_to_file(const char *url, const char *local_path)
{
    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 10000,
    };
    esp_http_client_handle_t client = NULL;
    FILE *file = NULL;
    char buffer[HTTP_BUFFER_SIZE];
    int result = 1;

    client = esp_http_client_init(&config);
    if (client == NULL) {
        shell_printf("wget: failed to create HTTP client\n");
        return 1;
    }

    if (esp_http_client_open(client, 0) != ESP_OK) {
        shell_printf("wget: failed to open %s\n", url);
        goto cleanup;
    }

    file = fopen(local_path, "wb");
    if (file == NULL) {
        shell_printf("wget: cannot open %s: %s\n", shell_pretty_path(local_path), strerror(errno));
        goto cleanup;
    }

    while (true) {
        int bytes_read = esp_http_client_read(client, buffer, sizeof(buffer));

        if (bytes_read < 0) {
            shell_printf("wget: read error\n");
            goto cleanup;
        }
        if (bytes_read == 0) {
            break;
        }
        if (fwrite(buffer, 1, (size_t)bytes_read, file) != (size_t)bytes_read) {
            shell_printf("wget: write error to %s\n", shell_pretty_path(local_path));
            goto cleanup;
        }
    }

    if (esp_http_client_get_status_code(client) >= 400) {
        shell_printf("wget: HTTP %d\n", esp_http_client_get_status_code(client));
        goto cleanup;
    }

    shell_printf("Saved %s -> %s\n", url, shell_pretty_path(local_path));
    result = 0;

cleanup:
    if (file != NULL) {
        fclose(file);
        if (result != 0) {
            unlink(local_path);
        }
    }
    if (client != NULL) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
    }

    return result;
}

static int list_directory(const char *path)
{
    DIR *dir = opendir(path);
    struct dirent *entry;

    if (dir == NULL) {
        if (errno == ENOENT && strncmp(path, "/spiffs", 7) == 0) {
            return 0;
        }
        shell_printf("ls: cannot open %s: %s\n", shell_pretty_path(path), strerror(errno));
        return 1;
    }

    while ((entry = readdir(dir)) != NULL) {
        char child[SHELL_PATH_SIZE];
        struct stat st;

        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        if (snprintf(child, sizeof(child), "%s/%s", path, entry->d_name) >= (int)sizeof(child)) {
            continue;
        }
        if (stat(child, &st) == 0 && S_ISDIR(st.st_mode)) {
            shell_printf("%s/\n", entry->d_name);
        } else {
            shell_printf("%s\n", entry->d_name);
        }
    }

    closedir(dir);
    return 0;
}

static int cat_file(const char *path)
{
    FILE *file = fopen(path, "r");
    char buffer[128];
    bool ended_with_newline = true;

    if (file == NULL) {
        shell_printf("cat: cannot open %s: %s\n", shell_pretty_path(path), strerror(errno));
        return 1;
    }

    while (fgets(buffer, sizeof(buffer), file) != NULL) {
        size_t len = strlen(buffer);

        ended_with_newline = len > 0 && buffer[len - 1] == '\n';
        shell_write_raw(buffer, len);
    }
    if (!ended_with_newline) {
        shell_printf("\n");
    }

    fclose(file);
    return 0;
}

static char *trim_in_place(char *text)
{
    char *start = text;
    char *end;

    while (*start != '\0' && isspace((unsigned char)*start)) {
        start++;
    }
    if (*start == '\0') {
        return start;
    }

    end = start + strlen(start) - 1;
    while (end > start && isspace((unsigned char)*end)) {
        *end-- = '\0';
    }

    return start;
}

static const char *skip_spaces(const char *text)
{
    while (*text != '\0' && isspace((unsigned char)*text)) {
        text++;
    }
    return text;
}

static bool parse_python_string(const char **cursor, char *out, size_t out_size)
{
    const char *text = skip_spaces(*cursor);
    char quote;
    size_t length = 0;

    if (*text != '\'' && *text != '"') {
        return false;
    }

    quote = *text++;
    while (*text != '\0' && *text != quote) {
        if (*text == '\\' && text[1] != '\0') {
            text++;
        }
        if (length + 1 >= out_size) {
            return false;
        }
        out[length++] = *text++;
    }
    if (*text != quote) {
        return false;
    }

    out[length] = '\0';
    *cursor = text + 1;
    return true;
}

static bool parse_python_u32(const char **cursor, uint32_t *value)
{
    const char *text = skip_spaces(*cursor);
    char *end = NULL;
    unsigned long parsed;

    if (!isdigit((unsigned char)*text)) {
        return false;
    }

    parsed = strtoul(text, &end, 10);
    if (end == text) {
        return false;
    }

    *value = (uint32_t)parsed;
    *cursor = end;
    return true;
}

static bool python_expect_end(const char *cursor)
{
    cursor = skip_spaces(cursor);
    return *cursor == ')' || *cursor == '\0';
}

static int run_python_line(char *line)
{
    char *trimmed = trim_in_place(line);

    if (*trimmed == '\0' || *trimmed == '#') {
        return 0;
    }

    if (strncmp(trimmed, "import ", 7) == 0) {
        char imports[PY_STRING_ARG_SIZE];
        char *token;

        snprintf(imports, sizeof(imports), "%s", trimmed + 7);
        token = strtok(imports, ",");
        while (token != NULL) {
            char *name = trim_in_place(token);
            char *space = strchr(name, ' ');

            if (space != NULL) {
                *space = '\0';
            }
            if (strcmp(name, "sys") != 0 && strcmp(name, "time") != 0 && strcmp(name, "os") != 0) {
                shell_printf("python: unsupported import: %s\n", name);
                return 1;
            }
            token = strtok(NULL, ",");
        }
        return 0;
    }

    if (strncmp(trimmed, "from ", 5) == 0) {
        char module[32] = {0};

        if (sscanf(trimmed, "from %31s import", module) == 1) {
            if (strcmp(module, "sys") == 0 || strcmp(module, "time") == 0 || strcmp(module, "os") == 0) {
                return 0;
            }
            shell_printf("python: unsupported import: %s\n", module);
            return 1;
        }
    }

    if (strncmp(trimmed, "print(", 6) == 0) {
        const char *cursor = trimmed + 6;
        char text[PY_STRING_ARG_SIZE];

        if (!parse_python_string(&cursor, text, sizeof(text)) || !python_expect_end(cursor)) {
            printf("python: invalid print()\n");
            shell_printf("python: invalid print()\n");
            return 1;
        }
        shell_printf("%s\n", text);
        return 0;
    }

    if (strncmp(trimmed, "sleep(", 6) == 0) {
        const char *cursor = trimmed + 6;
        uint32_t delay_ms = 0;

        if (!parse_python_u32(&cursor, &delay_ms) || !python_expect_end(cursor)) {
            printf("python: invalid sleep()\n");
            shell_printf("python: invalid sleep()\n");
            return 1;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        return 0;
    }

    if (strncmp(trimmed, "pwd()", 5) == 0) {
        shell_printf("%s\n", shell_display_path());
        return 0;
    }

    if (strncmp(trimmed, "ls(", 3) == 0) {
        const char *cursor = trimmed + 3;
        char path_arg[PY_STRING_ARG_SIZE] = ".";
        char resolved[SHELL_PATH_SIZE];

        cursor = skip_spaces(cursor);
        if (*cursor != ')') {
            if (!parse_python_string(&cursor, path_arg, sizeof(path_arg)) || !python_expect_end(cursor)) {
                shell_printf("python: invalid ls()\n");
                return 1;
            }
        }
        if (shell_resolve_path(path_arg, resolved, sizeof(resolved)) != ESP_OK) {
            shell_printf("python: bad path %s\n", path_arg);
            return 1;
        }
        return list_directory(resolved);
    }

    if (strncmp(trimmed, "cat(", 4) == 0 || strncmp(trimmed, "mkdir(", 6) == 0 ||
        strncmp(trimmed, "rm(", 3) == 0 || strncmp(trimmed, "touch(", 6) == 0) {
        bool is_cat = strncmp(trimmed, "cat(", 4) == 0;
        bool is_mkdir = strncmp(trimmed, "mkdir(", 6) == 0;
        bool is_rm = strncmp(trimmed, "rm(", 3) == 0;
        const char *cursor = trimmed + (is_cat ? 4 : (is_mkdir ? 6 : (is_rm ? 3 : 6)));
        char path_arg[PY_STRING_ARG_SIZE];
        char resolved[SHELL_PATH_SIZE];

        if (!parse_python_string(&cursor, path_arg, sizeof(path_arg)) || !python_expect_end(cursor)) {
            shell_printf("python: invalid file call\n");
            return 1;
        }
        if (shell_resolve_path(path_arg, resolved, sizeof(resolved)) != ESP_OK) {
            shell_printf("python: bad path %s\n", path_arg);
            return 1;
        }
        if (is_cat) {
            return cat_file(resolved);
        }
        if (is_mkdir) {
            if (mkdir(resolved, 0775) != 0) {
                printf("mkdir: %s\n", strerror(errno));
                shell_printf("mkdir: %s\n", strerror(errno));
                return 1;
            }
            return 0;
        }
        if (is_rm) {
            struct stat st;

            if (stat(resolved, &st) != 0) {
                shell_printf("rm: %s\n", strerror(errno));
                return 1;
            }
            if ((S_ISDIR(st.st_mode) ? rmdir(resolved) : unlink(resolved)) != 0) {
                shell_printf("rm: %s\n", strerror(errno));
                return 1;
            }
            return 0;
        }
        FILE *file = fopen(resolved, "ab");

        if (file == NULL) {
            shell_printf("touch: %s\n", strerror(errno));
            return 1;
        }
        fclose(file);
        return 0;
    }

    if (strncmp(trimmed, "wget(", 5) == 0) {
        const char *cursor = trimmed + 5;
        char url[PY_STRING_ARG_SIZE];
        char dest[PY_STRING_ARG_SIZE];
        char resolved[SHELL_PATH_SIZE];

        if (!parse_python_string(&cursor, url, sizeof(url))) {
            shell_printf("python: invalid wget() url\n");
            return 1;
        }
        cursor = skip_spaces(cursor);
        if (*cursor != ',') {
            shell_printf("python: invalid wget()\n");
            return 1;
        }
        cursor++;
        if (!parse_python_string(&cursor, dest, sizeof(dest)) || !python_expect_end(cursor)) {
            shell_printf("python: invalid wget() dest\n");
            return 1;
        }
        if (shell_resolve_path(dest, resolved, sizeof(resolved)) != ESP_OK) {
            shell_printf("python: bad path %s\n", dest);
            return 1;
        }
        return wget_download_to_file(url, resolved);
    }

    shell_printf("python: unsupported line: %s\n", trimmed);
    return 1;
}

static int run_python_script_file(const char *resolved_path)
{
    FILE *file = fopen(resolved_path, "r");
    char line[192];
    int line_number = 0;

    if (file == NULL) {
        shell_printf("python: cannot open %s: %s\n", shell_pretty_path(resolved_path), strerror(errno));
        return 1;
    }

    while (fgets(line, sizeof(line), file) != NULL) {
        int rc;

        line_number++;
        rc = run_python_line(line);
        if (rc != 0) {
            shell_printf("python: failed at line %d\n", line_number);
            fclose(file);
            return rc;
        }
    }

    fclose(file);
    return 0;
}

static void status_led_set(bool on)
{
    gpio_set_level(STATUS_LED_PIN, on ? STATUS_LED_ON : !STATUS_LED_ON);
}

static const uint8_t *oled_lookup_glyph(char ch)
{
    if (ch >= 'a' && ch <= 'z') {
        ch = (char)(ch - ('a' - 'A'));
    }

    if (ch == ' ') {
        return oled_font[0];
    }
    if (ch >= 'A' && ch <= 'Z') {
        return oled_font[1 + (ch - 'A')];
    }
    if (ch >= '0' && ch <= '9') {
        return oled_font[27 + (ch - '0')];
    }
    if (ch == ':') {
        return oled_font[37];
    }
    if (ch == '.') {
        return oled_font[38];
    }
    if (ch == '-') {
        return oled_font[39];
    }
    if (ch == '/') {
        return oled_font[40];
    }

    return oled_font[0];
}

static void oled_clear(void)
{
    memset(oled_framebuffer, 0, sizeof(oled_framebuffer));
}

static bool oled_char_needs_vertical_flip(char ch)
{
    return (ch >= '0' && ch <= '9');
}

static uint8_t oled_reverse_bits(uint8_t value)
{
    value = (uint8_t)(((value & 0xF0U) >> 4) | ((value & 0x0FU) << 4));
    value = (uint8_t)(((value & 0xCCU) >> 2) | ((value & 0x33U) << 2));
    value = (uint8_t)(((value & 0xAAU) >> 1) | ((value & 0x55U) << 1));
    return value;
}

static void oled_draw_text(uint8_t line, const char *text)
{
    size_t start = line * OLED_WIDTH;
    size_t pos = 0;

    if (line >= OLED_PAGES) {
        return;
    }

    while (*text != '\0' && pos < OLED_LINE_CHARS) {
        char ch = *text++;
        const uint8_t *glyph = oled_lookup_glyph(ch);
        size_t offset = start + (pos * 6);

        for (size_t i = 0; i < 5; ++i) {
            oled_framebuffer[offset + i] = oled_char_needs_vertical_flip(ch)
                ? oled_reverse_bits(glyph[i])
                : glyph[i];
        }
        oled_framebuffer[offset + 5] = 0x00;
        pos++;
    }
}

static esp_err_t oled_send_buffer(void)
{
    uint8_t tx[OLED_WIDTH + 1];

    tx[0] = 0x40;
    for (uint8_t page = 0; page < OLED_PAGES; ++page) {
        memcpy(&tx[1], &oled_framebuffer[page * OLED_WIDTH], OLED_WIDTH);
        oled_send_cmd((uint8_t)(0xB0 + page));
        oled_send_cmd((uint8_t)(0x00 + (OLED_COLUMN_OFFSET & 0x0F)));
        oled_send_cmd((uint8_t)(0x10 + ((OLED_COLUMN_OFFSET >> 4) & 0x0F)));
        esp_err_t err = i2c_master_transmit(oled_handle, tx, sizeof(tx), -1);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

static void oled_draw_ip_plain(uint8_t line)
{
    char chunk[OLED_LINE_CHARS + 1];
    size_t ip_len = strnlen(sys_ip, sizeof(sys_ip));
    size_t first_len;

    if (line >= OLED_PAGES) {
        return;
    }

    if (!wifi_connected) {
        oled_draw_text(line, "NO WIFI");
        if (line + 1 < OLED_PAGES) {
            oled_draw_text(line + 1, wifi_status_text());
        }
        return;
    }

    first_len = ip_len > OLED_LINE_CHARS ? OLED_LINE_CHARS : ip_len;
    memset(chunk, 0, sizeof(chunk));
    memcpy(chunk, sys_ip, first_len);
    oled_draw_text(line, chunk);

    if (line + 1 < OLED_PAGES) {
        if (ip_len > first_len) {
            size_t second_len = ip_len - first_len;
            if (second_len > OLED_LINE_CHARS) {
                second_len = OLED_LINE_CHARS;
            }
            memset(chunk, 0, sizeof(chunk));
            memcpy(chunk, sys_ip + first_len, second_len);
            oled_draw_text(line + 1, chunk);
        } else {
            oled_draw_text(line + 1, "");
        }
    }
}

static void oled_render_dashboard(void)
{
    uint64_t uptime_ms = esp_timer_get_time() / 1000ULL;

    uint64_t page = (uptime_ms / 4000ULL) % 2ULL;
    oled_clear();
    if (page == 0ULL) {
        /* Screen 1: IP + Telnet status */
        oled_draw_text(0, "ASSIX-C3 OS");
        oled_draw_ip_plain(1);
        oled_draw_text(3, "");
        oled_draw_text(4, telnet_enabled ? "TELNET ON" : "TELNET OFF");
    } else {
        /* Screen 2: How to connect */
        oled_draw_text(0, "CONNECT");
        oled_draw_ip_plain(1);
        oled_draw_text(2, "");
        oled_draw_text(3, "USER " SHELL_USER);
        oled_draw_text(4, "PASS " SHELL_PASS);
    }

    oled_send_buffer();
}

// --- 1. Shell Commands ---

static int do_help(int argc, char **argv)
{
    size_t index;

    (void)argc;
    (void)argv;
    for (index = 0; index < shell_command_count; ++index) {
        shell_printf("%-8s %s\n", shell_commands[index].name, shell_commands[index].help);
    }

    return 0;
}

static int do_df(int argc, char **argv) {
    size_t total = 0, used = 0;

    (void)argc;
    (void)argv;
    esp_spiffs_info(NULL, &total, &used);
    shell_printf("Disk Usage: %d/%d KB\n", (int)used/1024, (int)total/1024);
    return 0;
}

static int do_echo(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        shell_printf("%s%s", argv[i], i + 1 < argc ? " " : "");
    }
    shell_printf("\n");
    return 0;
}

static int do_free(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    unsigned long free_now = (unsigned long)esp_get_free_heap_size();
    unsigned long free_min = (unsigned long)esp_get_minimum_free_heap_size();

    shell_printf("%-16s %8s\n", "", "bytes");
    shell_printf("%-16s %8lu\n", "free:", free_now);
    shell_printf("%-16s %8lu\n", "free (min):", free_min);
    shell_printf("%-16s %8lu\n", "used (est):", 320UL * 1024UL - free_now);
    return 0;
}

static int do_uname(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    shell_printf("assix-c3-os ESP32-C3 v1.0 (ESP-IDF " IDF_VER ") RISC-V\n");
    return 0;
}

static int do_wifi(int argc, char **argv)
{
    size_t pass_len;
    char masked_pass[sizeof(WIFI_PASS) + 1];
    const char *state = "IDLE";

    (void)argc;
    (void)argv;

    switch (wifi_state) {
    case WIFI_STATE_CONNECTING:
        state = "CONNECTING";
        break;
    case WIFI_STATE_CONNECTED:
        state = "CONNECTED";
        break;
    case WIFI_STATE_FAILED:
        state = "FAILED";
        break;
    default:
        state = "IDLE";
        break;
    }

    pass_len = strlen(WIFI_PASS);
    memset(masked_pass, '*', pass_len);
    masked_pass[pass_len] = '\0';

    shell_printf("Wi-Fi configuration\n");
    shell_printf("  ssid:          %s\n", WIFI_SSID);
    shell_printf("  password:      %s (%u chars)\n", masked_pass, (unsigned)pass_len);
    shell_printf("  state:         %s\n", state);
    shell_printf("  connected:     %s\n", wifi_connected ? "yes" : "no");
    shell_printf("  ip:            %s\n", sys_ip);
    shell_printf("  retry count:   %d/%d\n", wifi_retry_count, WIFI_MAX_RETRIES);
    shell_printf("  status text:   %s\n", wifi_status_text());

    return 0;
}

static int do_ping(int argc, char **argv)
{
    struct addrinfo hints = {0};
    struct addrinfo *res = NULL;
    int count = 4;
    int sock = -1;
    int sent = 0, received = 0;
    char ip_str[16];

    if (argc < 2) {
        shell_printf("Usage: ping <host> [count]\n");
        return 1;
    }
    if (argc >= 3) {
        count = atoi(argv[2]);
        if (count < 1 || count > 20) count = 4;
    }

    if (!wifi_connected) {
        shell_printf("ping: no network\n");
        return 1;
    }

    hints.ai_family = AF_INET;
    if (getaddrinfo(argv[1], NULL, &hints, &res) != 0 || res == NULL) {
        shell_printf("ping: cannot resolve %s\n", argv[1]);
        return 1;
    }

    struct sockaddr_in *sa = (struct sockaddr_in *)res->ai_addr;
    inet_ntop(AF_INET, &sa->sin_addr, ip_str, sizeof(ip_str));

    sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (sock < 0) {
        shell_printf("ping: socket error %d\n", errno);
        freeaddrinfo(res);
        return 1;
    }

    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    shell_printf("PING %s (%s)\n", argv[1], ip_str);

    uint16_t pid = (uint16_t)(esp_random() & 0xFFFFU);

    for (int i = 1; i <= count; i++) {
        /* Build ICMP echo request */
        uint8_t pkt[44];
        memset(pkt, 0, sizeof(pkt));
        pkt[0] = 8;  /* type: echo request */
        pkt[1] = 0;  /* code */
        /* checksum @ [2..3] - computed below */
        pkt[4] = (uint8_t)(pid >> 8);
        pkt[5] = (uint8_t)(pid & 0xFF);
        pkt[6] = (uint8_t)(i >> 8);
        pkt[7] = (uint8_t)(i & 0xFF);
        memset(pkt + 8, 0xAB, sizeof(pkt) - 8);
        /* compute checksum */
        uint32_t sum = 0;
        for (size_t j = 0; j < sizeof(pkt); j += 2) {
            sum += (uint32_t)(((uint16_t)pkt[j] << 8) | pkt[j + 1]);
        }
        while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
        uint16_t csum = (uint16_t)(~sum);
        pkt[2] = (uint8_t)(csum >> 8);
        pkt[3] = (uint8_t)(csum & 0xFF);

        int64_t t0 = esp_timer_get_time();
        struct sockaddr_in dest = *sa;
        dest.sin_family = AF_INET;
        if (sendto(sock, pkt, sizeof(pkt), 0, (struct sockaddr *)&dest, sizeof(dest)) > 0) {
            sent++;
            uint8_t recv_buf[96];
            struct sockaddr_in from;
            socklen_t from_len = sizeof(from);
            while (1) {
                int rlen = recvfrom(sock, recv_buf, sizeof(recv_buf), 0,
                                   (struct sockaddr *)&from, &from_len);
                if (rlen < 0) break; /* timeout */
                /* IP header is typically 20 bytes; skip it */
                int ip_hlen = (recv_buf[0] & 0x0F) * 4;
                if (rlen < ip_hlen + 8) continue;
                uint8_t *icmp = recv_buf + ip_hlen;
                if (icmp[0] == 0 /* echo reply */
                    && ((uint16_t)(icmp[4] << 8) | icmp[5]) == pid
                    && ((uint16_t)(icmp[6] << 8) | icmp[7]) == (uint16_t)i) {
                    int64_t ms = (esp_timer_get_time() - t0) / 1000LL;
                    shell_printf("64 bytes from %s: seq=%d time=%lldms\n",
                                 ip_str, i, (long long)ms);
                    received++;
                    break;
                }
            }
        }
        if (i < count) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    close(sock);
    freeaddrinfo(res);
    shell_printf("%d packets transmitted, %d received\n", sent, received);
    return received > 0 ? 0 : 1;
}

static int do_wget(int argc, char **argv) {
    char resolved[SHELL_PATH_SIZE];

    if (argc < 2 || argc > 3) {
        shell_printf("Usage: wget <url> [local_file]\n");
        return 1;
    }

    if (argc == 2) {
        const char *slash = strrchr(argv[1], '/');
        const char *filename = (slash != NULL && slash[1] != '\0') ? slash + 1 : "download.bin";

        if (shell_resolve_path(filename, resolved, sizeof(resolved)) != ESP_OK) {
            shell_printf("wget: bad output path\n");
            return 1;
        }
    } else if (shell_resolve_path(argv[2], resolved, sizeof(resolved)) != ESP_OK) {
        shell_printf("wget: bad output path\n");
        return 1;
    }

    return wget_download_to_file(argv[1], resolved);
}

static int do_pwd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    shell_printf("%s\n", shell_display_path());
    return 0;
}

static int do_ls(int argc, char **argv)
{
    char resolved[SHELL_PATH_SIZE];
    const char *path = argc > 1 ? argv[1] : ".";

    if (shell_resolve_path(path, resolved, sizeof(resolved)) != ESP_OK) {
        shell_printf("ls: bad path %s\n", path);
        return 1;
    }

    return list_directory(resolved);
}

static int do_cd(int argc, char **argv)
{
    char resolved[SHELL_PATH_SIZE];
    const char *path = argc > 1 ? argv[1] : "/";

    if (shell_resolve_path(path, resolved, sizeof(resolved)) != ESP_OK) {
        shell_printf("cd: bad path %s\n", path);
        return 1;
    }
    if (chdir(resolved) != 0) {
        shell_printf("cd: %s: %s\n", path, strerror(errno));
        return 1;
    }

    return 0;
}

static int do_mkdir(int argc, char **argv)
{
    char resolved[SHELL_PATH_SIZE];

    if (argc != 2) {
        shell_printf("Usage: mkdir <path>\n");
        return 1;
    }
    if (shell_resolve_path(argv[1], resolved, sizeof(resolved)) != ESP_OK) {
        shell_printf("mkdir: bad path %s\n", argv[1]);
        return 1;
    }
    if (mkdir(resolved, 0775) != 0) {
        shell_printf("mkdir: %s\n", strerror(errno));
        return 1;
    }

    return 0;
}

static int do_touch(int argc, char **argv)
{
    char resolved[SHELL_PATH_SIZE];
    FILE *file;

    if (argc != 2) {
        shell_printf("Usage: touch <path>\n");
        return 1;
    }
    if (shell_resolve_path(argv[1], resolved, sizeof(resolved)) != ESP_OK) {
        shell_printf("touch: bad path %s\n", argv[1]);
        return 1;
    }

    file = fopen(resolved, "ab");
    if (file == NULL) {
        shell_printf("touch: %s\n", strerror(errno));
        return 1;
    }

    fclose(file);
    return 0;
}

static int do_cat(int argc, char **argv)
{
    char resolved[SHELL_PATH_SIZE];

    if (argc != 2) {
        shell_printf("Usage: cat <path>\n");
        return 1;
    }
    if (shell_resolve_path(argv[1], resolved, sizeof(resolved)) != ESP_OK) {
        shell_printf("cat: bad path %s\n", argv[1]);
        return 1;
    }

    return cat_file(resolved);
}

static int do_write_common(int argc, char **argv, bool append)
{
    char resolved[SHELL_PATH_SIZE];
    FILE *file;
    const char *mode = append ? "ab" : "wb";

    if (argc < 3) {
        shell_printf("Usage: %s <path> <text>\n", append ? "append" : "write");
        return 1;
    }
    if (shell_resolve_path(argv[1], resolved, sizeof(resolved)) != ESP_OK) {
        shell_printf("%s: bad path %s\n", append ? "append" : "write", argv[1]);
        return 1;
    }

    file = fopen(resolved, mode);
    if (file == NULL) {
        shell_printf("%s: %s\n", append ? "append" : "write", strerror(errno));
        return 1;
    }

    for (int i = 2; i < argc; i++) {
        if (i > 2) {
            fputc(' ', file);
        }
        fputs(argv[i], file);
    }
    fputc('\n', file);
    fclose(file);

    shell_printf("%s %s\n", append ? "Appended" : "Wrote", shell_pretty_path(resolved));
    return 0;
}

static int do_write(int argc, char **argv)
{
    return do_write_common(argc, argv, false);
}

static int do_append(int argc, char **argv)
{
    return do_write_common(argc, argv, true);
}

static int do_ip(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    shell_printf("IP: %s\n", sys_ip);
    shell_printf("TELNET: %s:%d\n", sys_ip, TELNET_PORT);
    shell_printf("Wi-Fi: %s\n", wifi_connected ? "connected" : "disconnected");
    return 0;
}

static int do_rm(int argc, char **argv)
{
    char resolved[SHELL_PATH_SIZE];
    struct stat st;

    if (argc != 2) {
        shell_printf("Usage: rm <path>\n");
        return 1;
    }
    if (shell_resolve_path(argv[1], resolved, sizeof(resolved)) != ESP_OK) {
        shell_printf("rm: bad path %s\n", argv[1]);
        return 1;
    }
    if (stat(resolved, &st) != 0) {
        shell_printf("rm: %s\n", strerror(errno));
        return 1;
    }
    if ((S_ISDIR(st.st_mode) ? rmdir(resolved) : unlink(resolved)) != 0) {
        shell_printf("rm: %s\n", strerror(errno));
        return 1;
    }

    return 0;
}

static int do_python(int argc, char **argv)
{
    char resolved[SHELL_PATH_SIZE];

    if (argc < 2 || argc > 3) {
        shell_printf("Usage: python <script.py> | python -c \"print('hi')\"\n");
        shell_printf("Supported calls: print, sleep, pwd, ls, cat, mkdir, rm, touch, wget\n");
        return 1;
    }

    if (argc == 3 && strcmp(argv[1], "-c") == 0) {
        char snippet[192];

        snprintf(snippet, sizeof(snippet), "%s", argv[2]);
        return run_python_line(snippet);
    }

    if (shell_resolve_path(argv[1], resolved, sizeof(resolved)) != ESP_OK) {
        shell_printf("python: bad path %s\n", argv[1]);
        return 1;
    }

    return run_python_script_file(resolved);
}

// --- 2. Hardware: OLED Dashboard ---

void oled_send_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    i2c_master_transmit(oled_handle, buf, 2, -1);
}

void init_oled() {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0, .sda_io_num = I2C_SDA_PIN, .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&bus_cfg, &bus_handle);
    i2c_device_config_t dev_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = OLED_ADDR, .scl_speed_hz = 400000 };
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &oled_handle);

    oled_send_cmd(0xAE);
    oled_send_cmd(0xD5);
    oled_send_cmd(0x80);
    oled_send_cmd(0xA8);
    oled_send_cmd(OLED_HEIGHT - 1);
    oled_send_cmd(0xD3);
    oled_send_cmd(0x00);
    oled_send_cmd(0x40);
    oled_send_cmd(0x8D);
    oled_send_cmd(0x14);
    oled_send_cmd(0x20);
    oled_send_cmd(0x00);
    oled_send_cmd(0xA0);
    oled_send_cmd(0xC0);
    oled_send_cmd(0xDA);
    oled_send_cmd(0x12);
    oled_send_cmd(0x81);
    oled_send_cmd(0x7F);
    oled_send_cmd(0xD9);
    oled_send_cmd(0xF1);
    oled_send_cmd(0xDB);
    oled_send_cmd(0x20);
    oled_send_cmd(0xA4);
    oled_send_cmd(0xA6);
    oled_send_cmd(0x2E);
    oled_send_cmd(0xAF);

    oled_render_dashboard();
}

void dashboard_task(void *pvParameters) {
    while (1) {
        oled_render_dashboard();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void led_task(void *pvParameters)
{
    bool led_on = false;

    (void)pvParameters;
    while (1) {
        if (wifi_state == WIFI_STATE_CONNECTING) {
            led_on = !led_on;
            status_led_set(led_on);
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        status_led_set(wifi_state == WIFI_STATE_FAILED);
        led_on = false;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// --- 3. Networking ---

static void wifi_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        wifi_state = WIFI_STATE_CONNECTING;
        wifi_retry_count = 0;
        esp_wifi_connect();
    } else if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
        wifi_connected = true;
        wifi_state = WIFI_STATE_CONNECTED;
        wifi_retry_count = 0;
        sprintf(sys_ip, IPSTR, IP2STR(&event->ip_info.ip));
        printf("\n[NET] IP Assigned: %s\n", sys_ip);
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        strcpy(sys_ip, "0.0.0.0");
        if (wifi_retry_count < WIFI_MAX_RETRIES) {
            wifi_retry_count++;
            wifi_state = WIFI_STATE_CONNECTING;
            esp_wifi_connect();
        } else {
            wifi_state = WIFI_STATE_FAILED;
            printf("\n[NET] Wi-Fi connect failed after %d attempts\n", wifi_retry_count);
        }
    }
}

// --- 4. Main OS ---

void app_main(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_handler, NULL, NULL);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler, NULL, NULL);

    init_oled();
    esp_vfs_spiffs_conf_t spiffs_cfg = { .base_path = "/spiffs", .partition_label = NULL, .max_files = 5, .format_if_mount_failed = true };
    esp_vfs_spiffs_register(&spiffs_cfg);
    mkdir("/spiffs", 0775);
    chdir("/spiffs");

    gpio_reset_pin(STATUS_LED_PIN);
    gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);
    status_led_set(false);

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_cfg = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();
    wifi_state = WIFI_STATE_CONNECTING;

    esp_console_config_t console_config = ESP_CONSOLE_CONFIG_DEFAULT();
    esp_console_init(&console_config);
    shell_mutex = xSemaphoreCreateMutex();

    xTaskCreate(dashboard_task, "dash", 4096, NULL, 5, NULL);
    xTaskCreate(led_task, "led", 2048, NULL, 5, NULL);
    xTaskCreate(telnet_server_task, "telnet", 6144, NULL, 5, NULL);

    printf("\n--- assix-OS Shell Box v0.9 ---\n");
    printf("Telnet User: %s | Pass: %s\n", SHELL_USER, SHELL_PASS);

    char line[128];
    int pos = 0;
    
    // Initial Prompt
    print_prompt();

    while(1) {
        int c = getchar();
        
        // Handle non-blocking noise
        if (c == EOF || c == 0xFFFF || c == -1) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (c == '\n' || c == '\r') {
            line[pos] = '\0';
            printf("\n");
            
            if (pos > 0) {
                shell_execute_line(line, shell_stdout_writer, NULL);
            }
            
            // Re-print prompt only AFTER processing
            pos = 0;
            print_prompt();
        } else if (c == '\b' || c == 0x7f) {
            if (pos > 0) {
                pos--;
                printf("\b \b");
                fflush(stdout);
            }
        } else if (pos < sizeof(line) - 1) {
            line[pos++] = (char)c;
            putchar(c);
            fflush(stdout);
        }
    }
}