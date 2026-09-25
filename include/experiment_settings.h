#ifndef EXPERIMENT_SETTINGS_H
#define EXPERIMENT_SETTINGS_H

#include <string>
#include <utility>
#include <vector>

#include "garment_session.h"

// Per-garment manual edits and optional Space+drag constraint paths.
struct ExperimentGarmentSettings
{
    // Batch parameters (stored separately per garment; deterministic given mesh + params).
    double maxCompression = -0.05;
    double maxTension = 0.03;

    // Border seam endpoints (vertex indices on the deformed mesh after load).
    std::vector<std::pair<size_t, size_t>> borderSeamEndpoints;
    std::vector<std::vector<int>> manualSegmentPatchSets;
    std::vector<std::vector<GarmentSession::CoordType>> constraintPaths;
};

// Full dual-garment experiment (meshes + batch parameters + manual edits).
struct ExperimentSettings
{
    static constexpr int kVersion = 2;

    std::string experimentDir;   // directory containing experiment.json
    std::string meshRelPathA;    // relative to experimentDir
    std::string meshRelPathB;
    ExperimentGarmentSettings garmentA;
    ExperimentGarmentSettings garmentB;
};

ExperimentGarmentSettings CollectExperimentGarmentSettings(const GarmentSession &session);

// Config file name inside an experiment folder.
extern const char kExperimentConfigFileName[];

// Root folder for saved experiments: <parafashion_project>/exp
extern const char kExperimentsRootDirName[];

// Directory containing CMakeLists.txt, src/, segmentMappingSolver/, etc.
std::string GetParafashionProjectDirectory();

std::string GetExperimentsRootDirectory(bool createIfMissing = true);

// Subfolder names under exp/ that contain a valid config (sorted alphabetically).
std::vector<std::string> ListSavedExperimentNames();

// Absolute path for <exp>/<name>, or empty if not a valid experiment folder.
std::string ResolveExperimentFolderPath(const std::string &nameOrPath);

// Segment-mapping outputs (polygons, mapping, quads). When an experiment is active,
// files are written under that folder using stable names (segment_mapping.txt, ...).
struct SegmentMappingArtifactPaths
{
    std::string polygonsA;
    std::string polygonsB;
    std::string mapping;
    std::string quads;
};

void SetActiveExperimentDirectory(const std::string &experimentDir);
void ClearActiveExperimentDirectory();
bool HasActiveExperimentDirectory();
std::string GetActiveExperimentDirectory();

// <active experiment>/<subdir>, or next to segment-mapping artifacts.
std::string ResolveExperimentExportDirectory(const std::string &meshPathA,
                                             const std::string &meshPathB,
                                             const std::string &subdir);

// Fitted approx polygons from segment mapping (quad export).
std::string ResolveFabricationApproxExportDirectory(const std::string &meshPathA,
                                                    const std::string &meshPathB);

// Seam-endpoint approx polygons per flattened segment.
std::string ResolveSegmentApproxExportDirectory(const std::string &meshPathA,
                                                const std::string &meshPathB);

SegmentMappingArtifactPaths SegmentMappingArtifactsForMeshes(const std::string &meshPathA,
                                                             const std::string &meshPathB);

// Base name of a mesh path without extension (e.g. "shirt" from "/path/shirt.obj").
std::string GarmentBaseNameFromPath(const std::string &meshPath);

// Creates <parentDir>/<nameA>_<nameB>[/ _N], copies meshes, writes config.json.
// Does not overwrite an existing folder; appends _1, _2, ... instead.
// On success, outExperimentDir is the created folder path.
bool ExportExperimentSettings(const std::string &parentDir,
                              GarmentSession &sessionA,
                              GarmentSession &sessionB,
                              const std::string &meshPathA,
                              const std::string &meshPathB,
                              double maxCompressionA,
                              double maxTensionA,
                              double maxCompressionB,
                              double maxTensionB,
                              std::string &outExperimentDir);

// Load config from a file path, or from a folder (looks for config.json / experiment.json).
bool LoadExperimentSettings(const std::string &path, ExperimentSettings &out);

void ApplyManualBorderSeamsFromEndpoints(
    GarmentSession &session,
    const std::vector<std::pair<size_t, size_t>> &endpoints);

// Run batch, re-apply border seams, then restore manual segment patch sets.
void RunBatchAndApplyManualEdits(GarmentSession &session,
                                 const ExperimentGarmentSettings &manual);

void NormalizeDualGarmentAreas(GarmentSession &sessionA, GarmentSession &sessionB);

// Load meshes from experiment package, normalize, set params, run batch on both garments.
bool ApplyExperimentSettings(GarmentSession &sessionA,
                             GarmentSession &sessionB,
                             const ExperimentSettings &exp,
                             std::string &outMeshPathA,
                             std::string &outMeshPathB);

#endif // EXPERIMENT_SETTINGS_H
