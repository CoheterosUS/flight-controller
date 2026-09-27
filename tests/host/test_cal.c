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
    Check(W25Q_CalAppend(M), "full sector rolls over");
    Check(W25Q_CalLoad(Loaded) && SameM(M, Loaded), "post-rollover calibration loads");
    Check(W25Q_CalGetSequence() == 65, "sequence continues after rollover");
    Check(flash_last_erase_address() == FLASH_CAL_SECTOR_B_ADDRESS,
          "rollover erases the other sector");
    Check(flash_bytes()[FLASH_CAL_SECTOR_A_ADDRESS] != 0xFF,
          "rollover preserves the old sector");
    Check(flash_bytes()[FLASH_CAL_SECTOR_B_ADDRESS + 64] == 0xFF,
          "rollover leaves later slots erased");
}

static void FillFirstSector(float M[9]) {
    for (int i = 0; i < 64; i++) {
        MakeM(M, (float)i + 100.0f);
        Check(W25Q_CalAppend(M), "fill first ping-pong sector");
    }
}

static void TestPowerLossDuringRolloverErase(void) {
    float M[9];
    float Old[9];
    float Loaded[9];

    flash_reset();
    FillFirstSector(Old);
    MakeM(M, 500.0f);
    flash_set_erase_limit(128);
    Check(!W25Q_CalAppend(M), "power loss during rollover erase fails append");
    flash_set_erase_limit(-1);
    Check(flash_last_erase_address() == FLASH_CAL_SECTOR_B_ADDRESS,
          "power-loss erase targets the other sector");
    Check(W25Q_CalLoad(Loaded) && SameM(Old, Loaded),
          "old record survives power loss during other-sector erase");
}

static void TestPowerLossAfterRolloverErase(void) {
    float M[9];
    float Old[9];
    float Loaded[9];

    flash_reset();
    FillFirstSector(Old);
    MakeM(M, 600.0f);
    flash_set_program_limit(0);
    Check(!W25Q_CalAppend(M), "power loss after rollover erase fails append");
    flash_set_program_limit(-1);
    Check(W25Q_CalLoad(Loaded) && SameM(Old, Loaded),
          "old record survives power loss after erase before program");
}

static void TestManyAppends(void) {
    float M[9];
    float Loaded[9];

    flash_reset();
    for (int i = 0; i < 220; i++) {
        MakeM(M, (float)i + 700.0f);
        Check(W25Q_CalAppend(M), "many ping-pong appends succeed");
        Check(W25Q_CalLoad(Loaded) && SameM(M, Loaded), "newest loads after every append");
    }
    Check(W25Q_CalGetSequence() == 220, "sequence continues across repeated rollovers");
}

static void TestTornSlotInSecondSector(void) {
    float M[9];
    float Torn[9];
    float Loaded[9];

    flash_reset();
    FillFirstSector(M);
    MakeM(M, 800.0f);
    Check(W25Q_CalAppend(M), "roll over before second-sector tear");
    MakeM(Torn, 801.0f);
    flash_set_program_limit(20);
    Check(!W25Q_CalAppend(Torn), "second-sector torn append fails");
    flash_set_program_limit(-1);
    MakeM(M, 802.0f);
    Check(W25Q_CalAppend(M), "append skips second-sector torn slot");
    Check(W25Q_CalLoad(Loaded) && SameM(M, Loaded), "newest survives tear in second sector");
    Check(W25Q_CalGetSequence() == 66, "tear does not consume a sequence number");
}

static void TestEraseAllRegionPreservesCalibration(void) {
    uint8_t *Flash;

    flash_reset();
    Flash = flash_bytes();
    Flash[0] = 0x12;
    Flash[W25Q_LOG_END - 1u] = 0x34;
    Flash[FLASH_CAL_SECTOR_A_ADDRESS] = 0x56;
    Flash[FLASH_CAL_SECTOR_B_ADDRESS] = 0x78;
    Check(W25Q_EraseLogRegion(W25Q_HANDLE) == HAL_OK, "erase-all log region succeeds");
    Check(Flash[0] == 0xFF && Flash[W25Q_LOG_END - 1u] == 0xFF,
          "erase-all clears everything below calibration sectors");
    Check(Flash[FLASH_CAL_SECTOR_A_ADDRESS] == 0x56
          && Flash[FLASH_CAL_SECTOR_B_ADDRESS] == 0x78,
          "erase-all preserves both calibration sectors");
}

static void TestPureRecoveryHelpers(void) {
    uint8_t Page[W25Q_PAGE_SIZE];

    memset(Page, 0xFF, sizeof(Page));
    Check(W25Q_PageIsFullyErased(Page), "fully erased candidate page is free");
    Page[37] = 0xFE;
    Check(!W25Q_PageIsFullyErased(Page), "programmed byte after erased first byte consumes page");
    Check(W25Q_ShouldCreateFlightMarker(false, false), "empty log needs a marker");
    Check(!W25Q_ShouldCreateFlightMarker(true, true), "last marker suppresses empty flight");
    Check(W25Q_ShouldCreateFlightMarker(true, false), "data after marker starts a real new flight");
}

static void TestSequenceCache(void) {
    float M[9];
    size_t Reads;

    flash_reset();
    MakeM(M, 900.0f);
    Check(W25Q_CalAppend(M), "append populates sequence cache");
    Reads = flash_read_count();
    Check(W25Q_CalGetSequence() == 1, "cached sequence is correct");
    Check(W25Q_CalGetSequence() == 1, "cached sequence remains correct");
    Check(flash_read_count() == Reads, "cached sequence performs no flash reads");
}

static void TestCrcCorruption(void) {
    float M[9];
    float Loaded[9];

    flash_reset();
    MakeM(M, 70.0f);
    Check(W25Q_CalAppend(M), "append before CRC corruption");
    flash_corrupt(FLASH_CAL_SECTOR_A_ADDRESS + 12,
                  (uint8_t)(flash_bytes()[FLASH_CAL_SECTOR_A_ADDRESS + 12] ^ 0x01u));
    Check(!W25Q_CalLoad(Loaded), "CRC corruption is detected");
}

static void TestLogBound(void) {
    Check(W25Q_LogHasSpaceAt(FLASH_CAL_SECTOR_A_ADDRESS - W25Q_PAGE_SIZE, W25Q_PAGE_SIZE), "last log page is available");
    Check(!W25Q_LogHasSpaceAt(FLASH_CAL_SECTOR_A_ADDRESS, 1), "calibration sectors are outside the log");
    Check(!W25Q_LogHasSpaceAt(FLASH_CAL_SECTOR_A_ADDRESS - 1, 2), "log cannot cross calibration boundary");
}

int main(void) {
    TestEmptyAndRoundTrip();
    TestNewestWins();
    TestTornRecordIsSkipped();
    TestFullSectorRollsOver();
    TestPowerLossDuringRolloverErase();
    TestPowerLossAfterRolloverErase();
    TestManyAppends();
    TestTornSlotInSecondSector();
    TestEraseAllRegionPreservesCalibration();
    TestPureRecoveryHelpers();
    TestSequenceCache();
    TestCrcCorruption();
    TestLogBound();

    if (Failures != 0) {
        fprintf(stderr, "%d host test(s) failed\n", Failures);
        return 1;
    }
    puts("host flash tests passed");
    return 0;
}
