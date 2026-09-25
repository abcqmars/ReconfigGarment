#include <QApplication>
#include <QWindow>
#include <clocale>
#include <iostream>

#include <GL/glew.h>
#include <wrap/qt/anttweakbarMapper.h>

#include "reconfigGarment_widget.h"

// Defined in src/reconfigGarment_widget.cpp
extern std::string pathMeshA;
extern std::string pathMeshB;
extern std::string gPendingExperimentJsonPath;

int main(int argc, char *argv[])
{
    // Use "." as decimal separator for consistency with other tools.
    std::setlocale(LC_NUMERIC, "en_US.UTF-8");

    if (argc >= 3 && std::string(argv[1]) == "--experiment") {
        gPendingExperimentJsonPath = std::string(argv[2]);
        std::cout << "Will load experiment from exp/: " << gPendingExperimentJsonPath << std::endl;
    } else if (argc >= 3) {
        pathMeshA = std::string(argv[1]);
        pathMeshB = std::string(argv[2]);
        std::cout << "Loading first garment mesh: " << pathMeshA << std::endl;
        std::cout << "Loading second garment mesh: " << pathMeshB << std::endl;
    } else {
        std::cerr << "Usage:\n"
                  << "  reconfigGarment <garment_mesh_1> <garment_mesh_2>\n"
                  << "  reconfigGarment --experiment <name under exp/>" << std::endl;
        return 1;
    }

    QApplication app(argc, argv);

    // Initialize AntTweakBar similarly to the single-garment app.
    QWindow dummy;
    QString def_string = QString("GLOBAL fontscaling=%1").arg((int)dummy.devicePixelRatio());
    TwDefine(def_string.toStdString().c_str());
    printf("%s\n", qPrintable(def_string));
    fflush(stdout);

    TwCopyCDStringToClientFunc(CopyCDStringToClient);
    TwCopyStdStringToClientFunc(CopyStdStringToClient);

    if (!TwInit(TW_OPENGL, nullptr))
    {
        fprintf(stderr, "AntTweakBar initialization failed: %s\n", TwGetLastError());
        return 1;
    }

    ReconfigGarmentWidget window;
    window.show();

    return app.exec();
}

