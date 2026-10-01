#include "experiment_settings.h"

#include <nlohmann/json.hpp>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#include <vcg/complex/algorithms/stat.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/position.h>

const char kExperimentConfigFileName[] = "config.json";
const char kExperimentsRootDirName[] = "exp";

namespace {

using json = nlohmann::json;

std::string sanitizeFolderComponent(std::string name)
{
    for (char &c : name) {
        if (c == '/' || c == '\\' || c == ':' || c == '*' || c == '?' || c == '"' ||
            c == '<' || c == '>' || c == '|' || c == ' ')
            c = '_';
    }
    while (!name.empty() && name.back() == '_')
        name.pop_back();
    while (!name.empty() && name.front() == '_')
        name.erase(name.begin());
    if (name.empty())
        name = "garment";
    return name;
}

std::string uniqueExperimentFolderPath(const std::string &parentDir,
                                       const std::string &meshPathA,
                                       const std::string &meshPathB)
{
    const std::string nameA = sanitizeFolderComponent(GarmentBaseNameFromPath(meshPathA));
    const std::string nameB = sanitizeFolderComponent(GarmentBaseNameFromPath(meshPathB));
    const std::string baseName = nameA + "_" + nameB;

    QDir parent(QString::fromStdString(parentDir));
    QString candidate = parent.filePath(QString::fromStdString(baseName));
    if (!parent.exists(candidate))
        return candidate.toStdString();

    for (int suffix = 1; suffix < 10000; ++suffix) {
        const std::string numbered = baseName + "_" + std::to_string(suffix);
        candidate = parent.filePath(QString::fromStdString(numbered));
        if (!parent.exists(candidate))
            return candidate.toStdString();
    }
    return std::string();
}

bool copyFileToDir(const std::string &srcPath,
                   const std::string &destDir,
                   const std::string &destFileName,
                   std::string &outRelName)
{
    QFileInfo srcInfo(QString::fromStdString(srcPath));
    if (!srcInfo.exists() || !srcInfo.isFile()) {
        std::cout << "Experiment export: source mesh not found: " << srcPath << std::endl;
        return false;
    }

    outRelName = destFileName.empty() ? srcInfo.fileName().toStdString() : destFileName;
    const QString destPath = QDir(QString::fromStdString(destDir)).filePath(QString::fromStdString(outRelName));
    if (QFile::exists(destPath)) {
        std::cout << "Experiment export: refusing to overwrite " << destPath.toStdString() << std::endl;
        return false;
    }
    if (!QFile::copy(QString::fromStdString(srcPath), destPath)) {
        std::cout << "Experiment export: failed to copy " << srcPath << " to " << destPath.toStdString()
                  << std::endl;
        return false;
    }
    return true;
}

bool experimentFolderHasConfig(const std::string &experimentDir)
{
    QDir dir(QString::fromStdString(experimentDir));
    if (QFile::exists(dir.filePath(QString::fromLatin1(kExperimentConfigFileName))))
        return true;
    return QFile::exists(dir.filePath("experiment.json"));
}

std::string configPathInExperimentDir(const std::string &experimentDir)
{
    QDir dir(QString::fromStdString(experimentDir));
    const QString config = dir.filePath(QString::fromLatin1(kExperimentConfigFileName));
    if (QFile::exists(config))
        return config.toStdString();
    const QString legacy = dir.filePath("experiment.json");
    if (QFile::exists(legacy))
        return legacy.toStdString();
    return config.toStdString();
}

json coordPathsToJson(const std::vector<std::vector<GarmentSession::CoordType>> &paths)
{
    json arr = json::array();
    for (const auto &path : paths) {
        json pathJson = json::array();
        for (const auto &p : path) {
            pathJson.push_back(json::array({p.X(), p.Y(), p.Z()}));
        }
        arr.push_back(std::move(pathJson));
    }
    return arr;
}

std::vector<std::vector<GarmentSession::CoordType>> coordPathsFromJson(const json &arr)
{
    std::vector<std::vector<GarmentSession::CoordType>> paths;
    if (!arr.is_array())
        return paths;

    for (const auto &pathJson : arr) {
        if (!pathJson.is_array())
            continue;
        std::vector<GarmentSession::CoordType> path;
        for (const auto &pt : pathJson) {
            if (!pt.is_array() || pt.size() < 3)
                continue;
            path.emplace_back(pt[0].get<double>(), pt[1].get<double>(), pt[2].get<double>());
        }
        if (!path.empty())
            paths.push_back(std::move(path));
    }
    return paths;
}

json borderSeamsToJson(const std::vector<std::pair<size_t, size_t>> &seams)
{
    json arr = json::array();
    for (const auto &seam : seams) {
        arr.push_back(json::array({seam.first, seam.second}));
    }
    return arr;
}

std::vector<std::pair<size_t, size_t>> borderSeamsFromJson(const json &arr)
{
    std::vector<std::pair<size_t, size_t>> seams;
    if (!arr.is_array())
        return seams;

    for (const auto &entry : arr) {
        if (!entry.is_array() || entry.size() < 2)
            continue;
        seams.emplace_back(entry[0].get<size_t>(), entry[1].get<size_t>());
    }
    return seams;
}

json patchSetsToJson(const std::vector<std::vector<int>> &sets)
{
    json arr = json::array();
    for (const auto &set : sets) {
        json setJson = json::array();
        for (int id : set)
            setJson.push_back(id);
        arr.push_back(std::move(setJson));
    }
    return arr;
}

std::vector<std::vector<int>> patchSetsFromJson(const json &arr)
{
    std::vector<std::vector<int>> sets;
    if (!arr.is_array())
        return sets;

    for (const auto &setJson : arr) {
        if (!setJson.is_array())
            continue;
        std::vector<int> set;
        for (const auto &id : setJson)
            set.push_back(id.get<int>());
        if (!set.empty())
            sets.push_back(std::move(set));
    }
    return sets;
}

json garmentSettingsToJson(const ExperimentGarmentSettings &g)
{
    return json{
        {"maxCompression", g.maxCompression},
        {"maxTension", g.maxTension},
        {"borderSeams", borderSeamsToJson(g.borderSeamEndpoints)},
        {"manualSegmentPatchSets", patchSetsToJson(g.manualSegmentPatchSets)},
        {"constraintPaths", coordPathsToJson(g.constraintPaths)},
    };
}

ExperimentGarmentSettings garmentSettingsFromJson(const json &obj)
{
    ExperimentGarmentSettings g;
    if (!obj.is_object())
        return g;
    g.maxCompression = obj.value("maxCompression", g.maxCompression);
    g.maxTension = obj.value("maxTension", g.maxTension);
    if (obj.contains("borderSeams"))
        g.borderSeamEndpoints = borderSeamsFromJson(obj["borderSeams"]);
    if (obj.contains("manualSegmentPatchSets"))
        g.manualSegmentPatchSets = patchSetsFromJson(obj["manualSegmentPatchSets"]);
    if (obj.contains("constraintPaths"))
        g.constraintPaths = coordPathsFromJson(obj["constraintPaths"]);
    return g;
}

std::string absoluteMeshPath(const ExperimentSettings &exp, const std::string &relPath)
{
    return QDir(QString::fromStdString(exp.experimentDir))
        .filePath(QString::fromStdString(relPath))
        .toStdString();
}

bool looksLikeParafashionProjectRoot(const QDir &dir)
{
    return QFile::exists(dir.filePath("CMakeLists.txt")) &&
           QFile::exists(dir.filePath("segmentMappingSolver")) &&
           QFile::exists(dir.filePath("src"));
}

} // namespace

std::string GetParafashionProjectDirectory()
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 10; ++depth) {
        if (looksLikeParafashionProjectRoot(dir))
            return dir.absolutePath().toStdString();
        if (!dir.cdUp())
            break;
    }

    // Same layout as segmentMappingSolver path: executable in build_dual/.
    QDir fallback(QCoreApplication::applicationDirPath());
    if (fallback.cdUp())
        return fallback.absolutePath().toStdString();
    return QCoreApplication::applicationDirPath().toStdString();
}

std::string GetExperimentsRootDirectory(bool createIfMissing)
{
    QDir project(QString::fromStdString(GetParafashionProjectDirectory()));
    const QString expPath = project.filePath(QString::fromLatin1(kExperimentsRootDirName));
    if (createIfMissing)
        project.mkpath(QString::fromLatin1(kExperimentsRootDirName));
    return expPath.toStdString();
}

std::vector<std::string> ListSavedExperimentNames()
{
    std::vector<std::string> names;
    QDir expDir(QString::fromStdString(GetExperimentsRootDirectory(false)));
    if (!expDir.exists())
        return names;

    const QFileInfoList entries =
        expDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    for (const QFileInfo &entry : entries) {
        if (!entry.isDir())
            continue;
        if (experimentFolderHasConfig(entry.absoluteFilePath().toStdString()))
            names.push_back(entry.fileName().toStdString());
    }
    std::sort(names.begin(), names.end());
    return names;
}

std::string ResolveExperimentFolderPath(const std::string &nameOrPath)
{
    QFileInfo info(QString::fromStdString(nameOrPath));
    if (info.isDir() && experimentFolderHasConfig(nameOrPath))
        return info.absoluteFilePath().toStdString();

    if (info.isDir())
        return std::string();

    const std::string underExp =
        QDir(QString::fromStdString(GetExperimentsRootDirectory(false)))
            .filePath(QString::fromStdString(nameOrPath))
            .toStdString();
    if (experimentFolderHasConfig(underExp))
        return underExp;
    return std::string();
}

namespace {

std::string gActiveExperimentDirectory;

std::string basePathFromMeshPath(const std::string &path)
{
    size_t pos = path.find_last_of('.');
    if (pos != std::string::npos)
        return path.substr(0, pos);
    return path;
}

} // namespace

void SetActiveExperimentDirectory(const std::string &experimentDir)
{
    gActiveExperimentDirectory = experimentDir;
}

void ClearActiveExperimentDirectory()
{
    gActiveExperimentDirectory.clear();
}

bool HasActiveExperimentDirectory()
{
    return !gActiveExperimentDirectory.empty();
}

std::string GetActiveExperimentDirectory()
{
    return gActiveExperimentDirectory;
}

std::string ResolveExperimentExportDirectory(const std::string &meshPathA,
                                             const std::string &meshPathB,
                                             const std::string &subdir)
{
    if (!gActiveExperimentDirectory.empty()) {
        QDir dir(QString::fromStdString(gActiveExperimentDirectory));
        return dir.filePath(QString::fromStdString(subdir)).toStdString();
    }

    const SegmentMappingArtifactPaths artifacts =
        SegmentMappingArtifactsForMeshes(meshPathA, meshPathB);
    if (!artifacts.mapping.empty()) {
        QFileInfo mapInfo(QString::fromStdString(artifacts.mapping));
        QDir parent = mapInfo.absoluteDir();
        if (parent.exists())
            return parent.filePath(QString::fromStdString(subdir)).toStdString();
    }

    const std::string expRoot = GetExperimentsRootDirectory(true);
    const std::string folder =
        sanitizeFolderComponent(GarmentBaseNameFromPath(meshPathA)) + "_" +
        sanitizeFolderComponent(GarmentBaseNameFromPath(meshPathB));
    QDir dir(QString::fromStdString(expRoot));
    return dir.filePath(QString::fromStdString(folder + "/" + subdir)).toStdString();
}

std::string ResolveFabricationApproxExportDirectory(const std::string &meshPathA,
                                                    const std::string &meshPathB)
{
    return ResolveExperimentExportDirectory(meshPathA, meshPathB, "fabrication_approx_polygons");
}

std::string ResolveSegmentApproxExportDirectory(const std::string &meshPathA,
                                                const std::string &meshPathB)
{
    return ResolveExperimentExportDirectory(meshPathA, meshPathB, "segment_approx_polygons");
}

SegmentMappingArtifactPaths SegmentMappingArtifactsForMeshes(const std::string &meshPathA,
                                                             const std::string &meshPathB)
{
    SegmentMappingArtifactPaths paths;

    if (!gActiveExperimentDirectory.empty()) {
        QDir dir(QString::fromStdString(gActiveExperimentDirectory));
        const std::string nameA = GarmentBaseNameFromPath(meshPathA);
        const std::string nameB = GarmentBaseNameFromPath(meshPathB);
        paths.polygonsA =
            dir.filePath(QString::fromStdString(nameA + "_polygons.txt")).toStdString();
        paths.polygonsB =
            dir.filePath(QString::fromStdString(nameB + "_polygons.txt")).toStdString();
        paths.mapping = dir.filePath(QString::fromLatin1("segment_mapping.txt")).toStdString();
        paths.quads = dir.filePath(QString::fromLatin1("segment_mapping_quads.txt")).toStdString();
        return paths;
    }

    const std::string baseA = basePathFromMeshPath(meshPathA);
    const std::string baseB = basePathFromMeshPath(meshPathB);
    paths.polygonsA = baseA + "_polygons.txt";
    paths.polygonsB = baseB + "_polygons.txt";
    paths.mapping = baseA + "_segment_mapping.txt";
    paths.quads = baseA + "_segment_mapping_quads.txt";
    return paths;
}

std::string GarmentBaseNameFromPath(const std::string &meshPath)
{
    QFileInfo info(QString::fromStdString(meshPath));
    QString base = info.completeBaseName();
    if (base.isEmpty())
        base = info.fileName();
    if (base.isEmpty())
        return "garment";
    return base.toStdString();
}

ExperimentGarmentSettings CollectExperimentGarmentSettings(const GarmentSession &session)
{
    ExperimentGarmentSettings g;
    g.constraintPaths = session.constraintPickedPoints;
    g.manualSegmentPatchSets = session.manualSegmentPatchSets;

    for (const auto &path : session.manualSeamVertexPaths) {
        if (path.empty())
            continue;
        g.borderSeamEndpoints.emplace_back(path.front(), path.back());
    }
    return g;
}

bool ExportExperimentSettings(const std::string &parentDir,
                              GarmentSession &sessionA,
                              GarmentSession &sessionB,
                              const std::string &meshPathA,
                              const std::string &meshPathB,
                              double maxCompressionA,
                              double maxTensionA,
                              double maxCompressionB,
                              double maxTensionB,
                              std::string &outExperimentDir)
{
    outExperimentDir.clear();
    const std::string experimentDir = uniqueExperimentFolderPath(parentDir, meshPathA, meshPathB);
    if (experimentDir.empty()) {
        std::cout << "Experiment export: could not allocate a unique folder name." << std::endl;
        return false;
    }

    if (!QDir().mkpath(QString::fromStdString(experimentDir))) {
        std::cout << "Experiment export: could not create " << experimentDir << std::endl;
        return false;
    }

    ExperimentSettings exp;
    exp.experimentDir = experimentDir;

    QFileInfo infoA(QString::fromStdString(meshPathA));
    QFileInfo infoB(QString::fromStdString(meshPathB));
    auto meshFileName = [](const QFileInfo &info, const std::string &baseName) -> std::string {
        const QString suffix = info.suffix();
        if (suffix.isEmpty())
            return baseName;
        return baseName + "." + suffix.toStdString();
    };
    const std::string destMeshA =
        meshFileName(infoA, sanitizeFolderComponent(GarmentBaseNameFromPath(meshPathA)));
    const std::string destMeshB =
        meshFileName(infoB, sanitizeFolderComponent(GarmentBaseNameFromPath(meshPathB)));

    if (!copyFileToDir(meshPathA, experimentDir, destMeshA, exp.meshRelPathA))
        return false;
    if (!copyFileToDir(meshPathB, experimentDir, destMeshB, exp.meshRelPathB))
        return false;

    exp.garmentA = CollectExperimentGarmentSettings(sessionA);
    exp.garmentB = CollectExperimentGarmentSettings(sessionB);
    exp.garmentA.maxCompression = maxCompressionA;
    exp.garmentA.maxTension = maxTensionA;
    exp.garmentB.maxCompression = maxCompressionB;
    exp.garmentB.maxTension = maxTensionB;

    json root;
    root["version"] = ExperimentSettings::kVersion;
    root["meshA"] = exp.meshRelPathA;
    root["meshB"] = exp.meshRelPathB;
    root["garmentA"] = garmentSettingsToJson(exp.garmentA);
    root["garmentB"] = garmentSettingsToJson(exp.garmentB);

    const std::string jsonPath =
        QDir(QString::fromStdString(experimentDir))
            .filePath(QString::fromLatin1(kExperimentConfigFileName))
            .toStdString();
    std::ofstream fout(jsonPath);
    if (!fout.is_open()) {
        std::cout << "Experiment export: could not write " << jsonPath << std::endl;
        return false;
    }
    fout << root.dump(2) << std::endl;
    outExperimentDir = experimentDir;
    std::cout << "Exported experiment to " << experimentDir << std::endl;
    return true;
}

bool LoadExperimentSettings(const std::string &path, ExperimentSettings &out)
{
    QFileInfo pathInfo(QString::fromStdString(path));
    std::string jsonPath = path;
    if (pathInfo.isDir())
        jsonPath = configPathInExperimentDir(path);

    std::ifstream fin(jsonPath);
    if (!fin.is_open()) {
        std::cout << "Experiment load: could not open " << jsonPath << std::endl;
        return false;
    }

    json root;
    try {
        fin >> root;
    } catch (const std::exception &e) {
        std::cout << "Experiment load: JSON parse error: " << e.what() << std::endl;
        return false;
    }

    const int version = root.value("version", 0);
    if (version != 1 && version != ExperimentSettings::kVersion) {
        std::cout << "Experiment load: unsupported version " << version
                  << " (supported: 1, " << ExperimentSettings::kVersion << ")" << std::endl;
        return false;
    }

    QFileInfo jsonInfo(QString::fromStdString(jsonPath));
    out.experimentDir = jsonInfo.absolutePath().toStdString();
    out.meshRelPathA = root.value("meshA", std::string("mesh_a.obj"));
    out.meshRelPathB = root.value("meshB", std::string("mesh_b.obj"));
    out.garmentA = root.contains("garmentA") ? garmentSettingsFromJson(root["garmentA"])
                                             : ExperimentGarmentSettings{};
    out.garmentB = root.contains("garmentB") ? garmentSettingsFromJson(root["garmentB"])
                                             : ExperimentGarmentSettings{};

    // Backward compatibility (v1): params were stored at the top level and shared.
    if (version == 1) {
        const double sharedCompr = root.value("maxCompression", -0.05);
        const double sharedTens = root.value("maxTension", 0.03);
        out.garmentA.maxCompression = sharedCompr;
        out.garmentA.maxTension = sharedTens;
        out.garmentB.maxCompression = sharedCompr;
        out.garmentB.maxTension = sharedTens;
    }

    return true;
}

void ApplyManualBorderSeamsFromEndpoints(
    GarmentSession &session,
    const std::vector<std::pair<size_t, size_t>> &endpoints)
{
    session.manualSeamVertexPaths.clear();
    session.manualBorderSeamPolylines.clear();

    for (const auto &seam : endpoints) {
        if (!AddManualBorderSeamToSession(session, seam.first, seam.second)) {
            std::cout << "Experiment: failed to apply border seam ("
                      << seam.first << ", " << seam.second << ")" << std::endl;
        }
    }
}

void RunBatchAndApplyManualEdits(GarmentSession &session,
                                 const ExperimentGarmentSettings &manual)
{
    const auto &picked = !manual.constraintPaths.empty()
                             ? manual.constraintPaths
                             : session.constraintPickedPoints;
    session.constraintPickedPoints = picked;

    DoBatchProcessWithPickedPoints(session, picked);

    ApplyManualBorderSeamsFromEndpoints(session, manual.borderSeamEndpoints);

    session.manualSegmentPatchSets = manual.manualSegmentPatchSets;
    session.manualSegmentPickInProgress.clear();
    if (!session.manualSegmentPatchSets.empty())
        RebuildGarmentSegments(session);
    else
        ColorSessionByPatch(session);
}

void NormalizeDualGarmentAreas(GarmentSession &sessionA, GarmentSession &sessionB)
{
    const double areaA = vcg::tri::Stat<TraceMesh>::ComputeMeshArea(sessionA.deformed_mesh);
    const double areaB = vcg::tri::Stat<TraceMesh>::ComputeMeshArea(sessionB.deformed_mesh);
    if (areaA <= 0.0 || areaB <= 0.0)
        return;

    vcg::tri::UpdatePosition<TraceMesh>::Scale(sessionB.deformed_mesh, std::sqrt(1000000.0 / areaB));
    vcg::tri::UpdateBounding<TraceMesh>::Box(sessionB.deformed_mesh);
    vcg::tri::UpdatePosition<TraceMesh>::Scale(sessionB.reference_mesh, std::sqrt(1000000.0 / areaB));
    vcg::tri::UpdateBounding<TraceMesh>::Box(sessionB.reference_mesh);

    vcg::tri::UpdatePosition<TraceMesh>::Scale(sessionA.deformed_mesh, std::sqrt(1000000.0 / areaA));
    vcg::tri::UpdateBounding<TraceMesh>::Box(sessionA.deformed_mesh);
    vcg::tri::UpdatePosition<TraceMesh>::Scale(sessionA.reference_mesh, std::sqrt(1000000.0 / areaA));
    vcg::tri::UpdateBounding<TraceMesh>::Box(sessionA.reference_mesh);

    sessionA.deformed_mesh.UpdateAttributes();
    sessionA.reference_mesh.UpdateAttributes();
    sessionB.deformed_mesh.UpdateAttributes();
    sessionB.reference_mesh.UpdateAttributes();
}

bool ApplyExperimentSettings(GarmentSession &sessionA,
                             GarmentSession &sessionB,
                             const ExperimentSettings &exp,
                             std::string &outMeshPathA,
                             std::string &outMeshPathB)
{
    outMeshPathA = absoluteMeshPath(exp, exp.meshRelPathA);
    outMeshPathB = absoluteMeshPath(exp, exp.meshRelPathB);

    if (!sessionA.loadMeshes(outMeshPathA, outMeshPathA, std::string())) {
        std::cout << "Experiment: failed to load garment A: " << outMeshPathA << std::endl;
        return false;
    }
    if (!sessionB.loadMeshes(outMeshPathB, outMeshPathB, std::string())) {
        std::cout << "Experiment: failed to load garment B: " << outMeshPathB << std::endl;
        return false;
    }

    NormalizeDualGarmentAreas(sessionA, sessionB);

    sessionA.initMesh();
    sessionB.initMesh();

    SetGarmentSessionTensionParams(sessionA, exp.garmentA.maxCompression, exp.garmentA.maxTension);
    SetGarmentSessionTensionParams(sessionB, exp.garmentB.maxCompression, exp.garmentB.maxTension);

    RunBatchAndApplyManualEdits(sessionA, exp.garmentA);
    RunBatchAndApplyManualEdits(sessionB, exp.garmentB);

    std::cout << "Experiment applied (batch + border seams + manual segments)." << std::endl;
    return true;
}
