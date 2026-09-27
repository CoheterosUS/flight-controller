#include "Sensors/W25Q32JV.h"
#include "flash_sim.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int Failures;

static void Check(int Condition, const char *Message) {
    if (!Condition) {
        fprintf(stderr, "FAIL: %s\n", Message);
        Failures++;
    }
}

static void MakeM(float M[9], float Base) {
    for (int i = 0; i < 9; i++) M[i] = Base + (float)i * 0.25f;
}

static int SameM(const float A[9], const float B[9]) {
    for (int i = 0; i < 9; i++) {
        if (fabsf(A[i] - B[i]) > 0.000001f) return 0;
    }
    return 1;
}

static void TestEmptyAndRoundTrip(void) {
    float M[9];
    float Loaded[9];

    flash_reset();
    Check(!W25Q_CalLoad(Loaded), "empty flash is not a calibration");
    MakeM(M, 1.0f);
    Check(W25Q_CalAppend(M), "first calibration append succeeds");
    Check(W25Q_CalLoad(Loaded) && SameM(M, Loaded), "calibration round trip");
    Check(W25Q_CalGetSequence() == 1, "first sequence is one");
    Check(flash_lock_count() > 0 && flash_lock_count() == flash_unlock_count(),
          "calibration operations use a balanced flash lock");
}

static void TestNewestWins(void) {
    float M[9];
    float Loaded[9];

    flash_reset();
    for (int i = 0; i < 4; i++) {
        MakeM(M, (float)i + 2.0f);
        Check(W25Q_CalAppend(M), "multiple calibration append succeeds");
    }
    Check(W25Q_CalLoad(Loaded) && SameM(M, Loaded), "newest valid calibration wins");
    Check(W25Q_CalGetSequence() == 4, "newest sequence is returned");
}

static void TestTornRecordIsSkipped(void) {
    float M[9];
    float Torn[9];
    float Loaded[9];

    flash_reset();
    MakeM(M, 10.0f);
    Check(W25Q_CalAppend(M), "append before torn record");
    MakeM(M, 20.0f);
    Check(W25Q_CalAppend(M), "second append before torn record");
    MakeM(Torn, 30.0f);
    flash_set_program_limit(12);
    Check(!W25Q_CalAppend(Torn), "power loss makes append fail");
    flash_set_program_limit(-1);
    MakeM(M, 40.0f);
    Check(W25Q_CalAppend(M), "append skips torn slot");
    Check(W25Q_CalLoad(Loaded) && SameM(M, Loaded), "torn record does not win");
    Check(W25Q_CalGetSequence() == 3, "sequence continues past torn record");
}

static void TestFullSectorRollsOver(void) {
    float M[9];
    float Loaded[9];

    flash_reset();
    for (int i = 0; i < 64; i++) {
        MakeM(M, (float)i + 50.0f);
        Check(W25Q_CalAppend(M), "fill calibration sector");
    }
    MakeM(M, 999.0f);
    Check(W25Q_CalAppend(M), "full sector erases and rewrites");
    Check(W25Q_CalLoad(Loaded) && SameM(M, Loaded), "post-rollover calibration loads");
    Check(W25Q_CalGetSequence() == 65, "sequence continues after rollover");
    Check(flash_bytes()[FLASH_CAL_SECTOR_ADDRESS + 64] == 0xFF, "rollover leaves later slots erased");
}

static void TestCrcCorruption(void) {
    float M[9];
    float Loaded[9];

    flash_reset();
    MakeM(M, 70.0f);
    Check(W25Q_CalAppend(M), "append before CRC corruption");
    flash_corrupt(FLASH_CAL_SECTOR_ADDRESS + 12, (uint8_t)(flash_bytes()[FLASH_CAL_SECTOR_ADDRESS + 12] ^ 0x01u));
    Check(!W25Q_CalLoad(Loaded), "CRC corruption is detected");
}

static void TestLogBound(void) {
    Check(W25Q_LogHasSpaceAt(FLASH_CAL_SECTOR_ADDRESS - W25Q_PAGE_SIZE, W25Q_PAGE_SIZE), "last log page is available");
    Check(!W25Q_LogHasSpaceAt(FLASH_CAL_SECTOR_ADDRESS, 1), "calibration sector is outside the log");
    Check(!W25Q_LogHasSpaceAt(FLASH_CAL_SECTOR_ADDRESS - 1, 2), "log cannot cross calibration boundary");
}

int main(void) {
    TestEmptyAndRoundTrip();
    TestNewestWins();
    TestTornRecordIsSkipped();
    TestFullSectorRollsOver();
    TestCrcCorruption();
    TestLogBound();

    if (Failures != 0) {
        fprintf(stderr, "%d host test(s) failed\n", Failures);
        return 1;
    }
    puts("host flash tests passed");
    return 0;
}
