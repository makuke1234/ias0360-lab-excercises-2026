#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "pico/util/queue.h"
#include "hardware/watchdog.h"

#include "ff.h"
#include "sd_card.h"
#include "f_util.h"
#include "hw_config.h"
#include "LCD_Driver.h"
#include "LCD_Touch.h"
#include "LCD_GUI.h"
#include "DEV_Config.h"
#include "icm20948.h"

#define PATH_MAX_LEN 256
#define LINE_LEN 128
#define SAMPLE_BUF_LEN 256

#define IMU_READY_FLAG 0x0000001

typedef struct
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint32_t t_us; // timestamp (lower 32 bits is fine for short runs)
} Sample;

mutex_t mutex;
TP_DATA tp_data;
Sample imu_data[SAMPLE_BUF_LEN];

// --------- Globals (FatFs requires the FS to outlive the mount) ----------
static FATFS fs;                   // must be static/global (lives as long as the mount)
static sd_card_t *g_sd = NULL;     // active SD card
static const char *g_drive = NULL; // typically "0:"

// ------------------------- Utility / Error -------------------------------
static void die(FRESULT fr, const char *op)
{
    printf("%s failed: %s (%d)\n", op, FRESULT_str(fr), fr);
    multicore_fifo_push_blocking(WRITE_FAILED_FLAG);
    while (1)
        tight_loop_contents();
}

static void loop_forever_msg(const char *msg)
{
    printf("%s\n", msg);
    while (1)
        tight_loop_contents();
}

static void join_path(char *out, size_t out_sz, const char *drive, const char *rel)
{
    // drive = "0:" or "0:/", ensure exactly one slash when joining
    if (rel && rel[0] == '/')
        rel++; // avoid double slashes
    if (drive && drive[strlen(drive) - 1] == '/')
        snprintf(out, out_sz, "%s%s", drive, rel ? rel : "");
    else
        snprintf(out, out_sz, "%s/%s", drive, rel ? rel : "");
}

// ------------------------- 1) Initialization -----------------------------
static bool sd_init_and_mount(void)
{
    if (!sd_init_driver())
    {
        printf("sd_init_driver() failed\n");
        return false;
    }

    g_sd = sd_get_by_num(0);
    if (!g_sd)
    {
        printf("No SD config found (sd_get_by_num(0) == NULL)\n");
        return false;
    }

    g_drive = sd_get_drive_prefix(g_sd); // usually "0:"
    if (!g_drive)
    {
        printf("sd_get_drive_prefix() returned NULL\n");
        return false;
    }

    FRESULT fr = f_mount(&fs, g_drive, 1);
    printf("f_mount -> %s (%d)\n", FRESULT_str(fr), fr);

    if (fr == FR_NO_FILESYSTEM)
    {
        BYTE work[4096]; // >= FF_MAX_SS
        MKFS_PARM opt = {FM_FAT | FM_SFD, 0, 0, 0, 0};
        fr = f_mkfs(g_drive, &opt, work, sizeof work);
        printf("f_mkfs -> %s (%d)\n", FRESULT_str(fr), fr);
        if (fr == FR_OK)
        {
            fr = f_mount(&fs, g_drive, 1);
            printf("f_mount(after mkfs) -> %s (%d)\n", FRESULT_str(fr), fr);
        }
    }

    if (fr != FR_OK)
    {
        printf("Mount failed: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }

    return true;
}

// ------------------------- 2) File creation ------------------------------
static FRESULT create_file(const char *abs_path, FIL *out_file)
{
    // Creates/truncates a file and opens it for writing
    return f_open(out_file, abs_path, FA_WRITE | FA_CREATE_ALWAYS);
}

static FRESULT append_file(const char *abs_path, FIL *out_file)
{
    return f_open(out_file, abs_path, FA_WRITE | FA_OPEN_APPEND);
}

// ------------------------- 3) File writing -------------------------------
static FRESULT write_to_file(FIL *file, const void *data, UINT len, UINT *bytes_written)
{
    *bytes_written = 0;

    printf("write_to_file: About to write %u bytes from pointer %p\n", len, data);
    printf("write_to_file: First few bytes: %02X %02X %02X %02X\n",
           ((uint8_t *)data)[0], ((uint8_t *)data)[1],
           ((uint8_t *)data)[2], ((uint8_t *)data)[3]);

    FRESULT fr = f_write(file, data, len, bytes_written);
    printf("write_to_file: f_write returned FR=%d, wrote %u bytes\n", fr, *bytes_written);

    if (fr == FR_OK)
    {
        fr = f_sync(file); // ensure data hits the card
        printf("write_to_file: f_sync returned FR=%d\n", fr);
    }

    return fr;
}

// ------------------------- 4) File checking/listing ----------------------
typedef struct
{
    uint32_t files;
    uint32_t dirs;
    uint64_t total_bytes;
} list_stats_t;

static bool is_dot_or_dotdot(const char *name)
{
    return (name[0] == '.' && (name[1] == '\0' || (name[1] == '.' && name[2] == '\0')));
}

static FRESULT list_dir_recursive(const char *path, list_stats_t *stats)
{
    DIR dir;
    FILINFO fno;
    FRESULT fr = f_opendir(&dir, path);
    if (fr != FR_OK)
    {
        printf("f_opendir('%s') -> %s (%d)\n", path, FRESULT_str(fr), fr);
        return fr;
    }

    for (;;)
    {
        fr = f_readdir(&dir, &fno);
        if (fr != FR_OK)
        {
            printf("f_readdir('%s') -> %s (%d)\n", path, FRESULT_str(fr), fr);
            break;
        }
        if (fno.fname[0] == '\0')
            break; // end of directory

        if (is_dot_or_dotdot(fno.fname))
            continue;

        if (fno.fattrib & AM_DIR)
        {
            stats->dirs++;
            char subpath[PATH_MAX_LEN];
            snprintf(subpath, sizeof subpath, "%s/%s", path, fno.fname);
            printf("[DIR]  %s\n", subpath);
            fr = list_dir_recursive(subpath, stats);
            if (fr != FR_OK)
                break;
        }
        else
        {
            stats->files++;
            stats->total_bytes += (uint64_t)fno.fsize;
            printf("[FILE] %s/%s  (%lu bytes)\n", path, fno.fname, (unsigned long)fno.fsize);
        }
    }

    FRESULT frc = f_closedir(&dir);
    if (fr == FR_OK && frc != FR_OK)
        fr = frc;
    return fr;
}

// Public checker: lists all files and sizes, and tells if any exist
static FRESULT check_and_list_files(const char *root_drive)
{
    // Build root path "0:/"
    char root[PATH_MAX_LEN];
    join_path(root, sizeof root, root_drive, ""); // ensures a trailing slash when we add children

    list_stats_t stats = {0};
    printf("\n--- SD Card File Listing for '%s' ---\n", root_drive);
    FRESULT fr = list_dir_recursive(root_drive, &stats);
    if (fr != FR_OK && fr != FR_NO_PATH)
    {
        printf("Directory listing aborted due to error.\n");
        return fr;
    }

    if (stats.files == 0 && stats.dirs == 0)
    {
        printf("No files or directories found on the SD card.\n");
    }
    else if (stats.files == 0)
    {
        printf("No files found (but %lu director%s present).\n", stats.dirs, (stats.dirs == 1 ? "y" : "ies"));
    }
    else
    {
        printf("\nSummary: %lu file%s in %lu director%s, total %llu bytes.\n",
               stats.files, (stats.files == 1 ? "" : "s"),
               stats.dirs, (stats.dirs == 1 ? "y" : "ies"),
               (unsigned long long)stats.total_bytes);
    }
    return FR_OK;
}

// ---------- Simple complex type ----------
typedef struct
{
    float re, im;
} c32;

// ---------- Small utilities ----------
static inline float fsqrtf(float x) { return sqrtf(x); }
static inline float fclampf(float v, float lo, float hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }

// ---------- Hamming window (in-place) ----------
static void hamming_window(float *x, int n)
{
    for (int i = 0; i < n; i++)
    {
        x[i] *= 0.54f - 0.46f * cosf(2.0f * (float)M_PI * (float)i / (float)(n - 1));
    }
}

// ---------- Bit helpers ----------
static int is_power_of_two(int n) { return (n > 0) && ((n & (n - 1)) == 0); }
static unsigned reverse_bits(unsigned v, int nbits)
{
    unsigned r = 0u;
    for (int i = 0; i < nbits; i++)
    {
        r = (r << 1) | (v & 1u);
        v >>= 1u;
    }
    return r;
}

// ---------- In-place radix-2 Cooley–Tukey FFT ----------
// dir = +1 for FFT, -1 for IFFT
static int fft_radix2(c32 *x, int n, int dir)
{
    if (!is_power_of_two(n))
        return -1;
    int logn = 0;
    while ((1 << logn) < n)
        logn++;

    // Bit-reversal permutation
    for (unsigned i = 0; i < (unsigned)n; i++)
    {
        unsigned j = reverse_bits(i, logn);
        if (j > i)
        {
            c32 t = x[i];
            x[i] = x[j];
            x[j] = t;
        }
    }

    const float sgn = (dir >= 0) ? -1.0f : 1.0f;
    for (int s = 1; s <= logn; s++)
    {
        int m = 1 << s;
        int m2 = m >> 1;
        float theta = sgn * (float)M_PI / (float)m2;
        float wpr = -2.0f * sinf(0.5f * theta) * sinf(0.5f * theta);
        float wpi = sinf(theta);
        for (int k = 0; k < n; k += m)
        {
            float wr = 1.0f, wi = 0.0f;
            for (int j = 0; j < m2; j++)
            {
                int t = k + j + m2;
                int u = k + j;
                float tr = wr * x[t].re - wi * x[t].im;
                float ti = wr * x[t].im + wi * x[t].re;
                float ur = x[u].re, ui = x[u].im;
                x[t].re = ur - tr;
                x[t].im = ui - ti;
                x[u].re = ur + tr;
                x[u].im = ui + ti;
                // twiddle update (CORDIC-free recurrence)
                float tmp = wr;
                wr = wr + (wr * wpr - wi * wpi);
                wi = wi + (wi * wpr + tmp * wpi);
            }
        }
    }

    if (dir < 0)
    {
        float inv = 1.0f / (float)n;
        for (int i = 0; i < n; i++)
        {
            x[i].re *= inv;
            x[i].im *= inv;
        }
    }
    return 0;
}

// ---------- Magnitude spectrum ----------
static void fft_mag(const c32 *X, int n, float *mag)
{
    for (int i = 0; i < n; i++)
    {
        mag[i] = fsqrtf(X[i].re * X[i].re + X[i].im * X[i].im);
    }
}

// ---------- Peak picking (top-K, single-sided) ----------
static void top_k_peaks(const float *mag, int n_half, int k_exclude_dc, int K,
                        int *out_idx, float *out_val, int *out_count)
{
    // simple selection without sorting the full array
    int count = 0;
    for (int k = 0; k < K; k++)
    {
        int best_i = -1;
        float best_v = -1.0f;
        for (int i = k_exclude_dc; i < n_half; i++)
        {
            // skip already taken
            bool taken = false;
            for (int j = 0; j < count; j++)
                if (out_idx[j] == i)
                {
                    taken = true;
                    break;
                }
            if (taken)
                continue;
            if (mag[i] > best_v)
            {
                best_v = mag[i];
                best_i = i;
            }
        }
        if (best_i >= 0)
        {
            out_idx[count] = best_i;
            out_val[count] = best_v;
            count++;
        }
    }
    *out_count = count;
}

void core1_entry()
{
    printf("Core 1 entry: write to SD card\n");
    sleep_ms(2000);

    // 1) Init + mount
    if (!sd_init_and_mount())
    {
        loop_forever_msg("SD init/mount failed.");
    }

    FRESULT fr;
    uint32_t t_prev = (uint32_t)time_us_64();

    while (1)
    {
        uint32_t msg = multicore_fifo_pop_blocking();

        if (msg != IMU_READY_FLAG)
        {
            printf("Core 1 received unexpected message: 0x%08lX\n", msg);
            continue;
        }

        printf("Writing data to a file\n");

        // Build absolute file path: <drive>/lcd_sd_card_example_<iteration>.txt
        char path[PATH_MAX_LEN];
        const char *name = "lab_1_2_imu_raw.txt";
        join_path(path, sizeof path, g_drive, name);

        printf("Core 1: Creating and writing to file: %s\n", path);
        // 2) Create the file
        FIL f;
        fr = append_file(path, &f);
        if (fr != FR_OK)
            die(fr, "f_open(create)");

        mutex_enter_blocking(&mutex);

        // 3) Write data
        UINT bw = 0;

        size_t max_len = LINE_LEN * SAMPLE_BUF_LEN + 1;
        char *ascii_buffer = (char *)malloc(max_len);
        ascii_buffer[0] = '\0';

        char *linebeg_ptr = ascii_buffer;
        for (uint8_t i = 0; i < SAMPLE_BUF_LEN; ++i)
        {
            Sample *s = &imu_data[i];
            uint32_t t_now = s->t_us;

            uint32_t dt_us = (t_now - t_prev);
            t_prev = t_now;

            float hz = (dt_us > 0) ? (1000000.0f / (float)dt_us) : 0.0f;

            snprintf(linebeg_ptr, LINE_LEN, "ACC: X=%d Y=%d Z=%d | GYRO: X=%d Y=%d Z=%d | Freq: %.1f Hz\r\n",
                     s->ax, s->ay, s->az, s->gx, s->gy, s->gz, hz);
            linebeg_ptr[LINE_LEN] = '\0';
            linebeg_ptr += strnlen(linebeg_ptr, LINE_LEN);
        }

        size_t data_len = strnlen(ascii_buffer, max_len);
        fr = write_to_file(&f, ascii_buffer, data_len, &bw);
        free(ascii_buffer);

        printf("Core 1: write_to_file returned FR=%d, bytes_written=%u (expected %zu)\n",
               fr, bw, data_len);
        if (fr != FR_OK || bw != data_len)
        {
            printf("ERROR: Write failed or incomplete! FR=%d, wrote %u/%zu bytes\n",
                   fr, bw, data_len);
            die(fr, "f_write/f_sync");
        }

        // Close the file
        f_close(&f);

        mutex_exit(&mutex);

        printf("----- File write iteration END -----\n");
    }

    // Optional: unmount
    fr = f_unmount(g_drive);
    printf("f_unmount -> %s (%d)\n", FRESULT_str(fr), fr);

    sleep_ms(1000);                                   // optional flush delay
    multicore_fifo_push_blocking(TASK_COMPLETE_FLAG); // acknowledge successful send

    printf("Core 1 task complete.\n");

    while (1)
    {
        tight_loop_contents();
    }
}

int main(void)
{
    System_Init();
    sleep_ms(3000);

    IMU_EN_SENSOR_TYPE type;
    imuInit(&type);

    if (IMU_EN_SENSOR_TYPE_ICM20948 == type)
    {
        printf("Motion sensor is ICM-20948 (multicore)\n");
    }
    else
    {
        printf("Motion sensor NULL\n");
    }

    mutex_init(&mutex); // Initialize the mutex
    multicore_launch_core1(core1_entry);

    Sample sample_buf[SAMPLE_BUF_LEN] = {0};
    uint8_t buf_idx = 0;

    while (1)
    {
        if (multicore_fifo_rvalid())
        {
            uint32_t msg = multicore_fifo_pop_blocking();
            if (msg == TASK_COMPLETE_FLAG)
            {
                printf("Core 0: Core 1 task complete.\n");
                break;
            }
            else if (msg == WRITE_FAILED_FLAG)
            {
                printf("Core 0: Core 1 reported write failure.\n");
                loop_forever_msg("Write failed on Core 1.");
            }
        }

        IMU_ST_SENSOR_DATA stGyroRawData, stAccelRawData;
        imuDataAccGyrGet(&stGyroRawData, &stAccelRawData);

        uint32_t t_now = (uint32_t)time_us_64();
        Sample s = {stAccelRawData.s16X, stAccelRawData.s16Y, stAccelRawData.s16Z,
                    stGyroRawData.s16X, stGyroRawData.s16Y, stGyroRawData.s16Z,
                    t_now};

        sample_buf[buf_idx] = s;
        ++buf_idx;
        if (buf_idx == SAMPLE_BUF_LEN)
        {
            mutex_enter_blocking(&mutex);
            memcpy(imu_data, sample_buf, sizeof(Sample) * SAMPLE_BUF_LEN);
            mutex_exit(&mutex);

            multicore_fifo_push_blocking(IMU_READY_FLAG);
            buf_idx = 0;
        }
    }

    printf("All tasks complete.\n");
    mutex_exit(&mutex);
    multicore_reset_core1();

    // // ---- Settings ----
    // const float fs = 2200.0f; // IMU sample rate (Hz)
    // enum
    // {
    //     N = 256
    // }; // power-of-two FFT size
    // const float df = fs / (float)N;

    // if (!is_power_of_two(N))
    // {
    //     printf("N must be power of two\n");
    //     while (true)
    //         tight_loop_contents();
    // }

    // // ---- Build a synthetic signal: 120 Hz and 440 Hz
    // float x[N];
    // for (int n = 0; n < N; n++)
    // {
    //     float t = (float)n / fs;
    //     x[n] = 0.7f * sinf(2.0f * (float)M_PI * 120.0f * t) + 0.3f * sinf(2.0f * (float)M_PI * 440.0f * t);
    // }

    // // ---- Window and pack into complex buffer
    // hamming_window(x, N);
    // c32 X[N];
    // for (int i = 0; i < N; i++)
    // {
    //     X[i].re = x[i];
    //     X[i].im = 0.0f;
    // }

    // // ---- FFT
    // if (fft_radix2(X, N, +1) != 0)
    // {
    //     printf("FFT error\n");
    //     while (true)
    //         tight_loop_contents();
    // }

    // // ---- Magnitude spectrum (single-sided bins 0..N/2)
    // float mag[N];
    // fft_mag(X, N, mag);

    // const float scale = (2.0f / (float)N) / 0.54f;
    // for (int k = 0; k <= N / 2; k++)
    //     mag[k] *= scale;

    // // ---- Report
    // printf("\n=== IMU FFT Demo ===\n");
    // printf("fs=%.1f Hz, N=%d, resolution df=%.3f Hz\n", fs, N, df);
    // printf("Looking for top 5 peaks (excluding DC):\n");

    // int idx[5];
    // float val[5];
    // int found = 0;
    // top_k_peaks(mag, N / 2 + 1, /*exclude up to index*/ 1, /*K*/ 5, idx, val, &found);

    // for (int i = 0; i < found; i++)
    // {
    //     float fk = df * (float)idx[i];
    //     printf("  Peak %d: bin=%d  freq=%.2f Hz  amplitude≈%.4f\n",
    //            i + 1, idx[i], fk, val[i]);
    // }

    // printf("\n");
    // printf("TO COPY ONTO A TEXT FILE FOR COMPARSION WITH PC RUN FFT\n");

    // // Print the raw values onto the serial in one line
    // for (int n = 0; n < N; n++)
    // {
    //     if (n)
    //         printf(",");
    //     printf("%.6f", (double)x[n]);
    // }
    // printf("\n");
    // printf("\n");
    // printf("TO COPY ONTO A TEXT FILE FOR COMPARSION WITH PC RUN FFT\n");

    // // Print to the serial the output of the fft to compare with the host run python implmentation of the fft filter
    // printf("bin,freq,amp\n");
    // for (int i = 0; i < found; i++)
    // {
    //     float fk = df * (float)idx[i];
    //     printf("%d,%.6f,%.6f\n", idx[i], fk, val[i]);
    // }

    // while (true)
    //     tight_loop_contents();
    // return 0;
}
