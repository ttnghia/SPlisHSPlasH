#include "SPlisHSPlasH/Common.h"
#include "GL/glew.h"
#include "Visualization/MiniGL.h"
#include "GL/glut.h"
#include "SPlisHSPlasH/TimeManager.h"
#include <Eigen/Dense>
#include <iostream>
#include "SPlisHSPlasH/Utilities/Timing.h"
#include "Utilities/PartioReaderWriter.h"
#include "SPlisHSPlasH/StaticRigidBody.h"
#include "Utilities/OBJLoader.h"
#include "SPlisHSPlasH/Utilities/PoissonDiskSampling.h"
#include "Demos/Common/DemoBase.h"
#include "Utilities/FileSystem.h"

// Enable memory leak detection
#ifdef _DEBUG
#ifndef EIGEN_ALIGN
#define new DEBUG_NEW
#endif
#endif

using namespace SPH;
using namespace Eigen;
using namespace std;

void timeStep();
void initBoundaryData();
void render();
void renderBoundary();
void reset();


DemoBase base;


// main
int main(int argc, char** argv)
{
    REPORT_MEMORY_LEAKS;

    base.init(argc, argv, "StaticBoundaryDemo");
    initBoundaryData();
    base.buildModel();

    ////////////////////////////////////////////////////////////////////////////////
    auto scene            = base.getScene();
    auto simulationMethod = base.getSimulationMethod();
    base.m_MeshWriter->reset_buffer();
    base.m_MeshWriter->getBuffer().push_back(static_cast<unsigned int>(scene.boundaryModels.size() - 1));

    static std::vector<Vector3r> vertices;
    static std::vector<Vector3r> normals;
    for(unsigned int i = 0; i < base.getScene().boundaryModels.size(); i++)
    {
        if(!scene.boundaryModels[i]->isWall)
        {
            FluidModel::RigidBodyParticleObject* rb  = simulationMethod.model.getRigidBodyParticleObject(i);
            RigidBodyObject*                     rbo = rb->m_rigidBody;
            StaticRigidBody*                     srb = dynamic_cast<StaticRigidBody*>(rbo);

            assert(srb != nullptr);
            TriangleMesh&      mesh   = srb->getGeometry();
            const unsigned int nFaces = mesh.numFaces();
            auto faces             = mesh.getFaces();
            auto faceVertices      = mesh.getVertices();
            auto faceVertexNormals = mesh.getVertexNormals();

            vertices.resize(0);
            vertices.reserve(nFaces * 3);
            normals.resize(0);
            normals.reserve(nFaces * 3);

            for(unsigned int i = 0; i < nFaces; ++i)
            {
                for(unsigned int j = 0; j < 3; ++j)
                {
                    unsigned int v_index = faces[i * 3 + j];
                    vertices.push_back(faceVertices[v_index]);
                    normals.push_back(faceVertexNormals[v_index]);
                }
            }

            base.m_MeshWriter->getBuffer().push_back(static_cast<unsigned int>(vertices.size()));
            base.m_MeshWriter->getBuffer().push_back_to_float_array(vertices, false);
            base.m_MeshWriter->getBuffer().push_back_to_float_array(normals, false);
        }
    }
    base.m_MeshWriter->flush_buffer_async(1);


    ////////////////////////////////////////////////////////////////////////////////
    MiniGL::setClientIdleFunc(50, timeStep);
    MiniGL::setKeyFunc(0, 'r', reset);
    MiniGL::setClientSceneFunc(render);

    glutMainLoop();

    base.cleanup();

    Timing::printAverageTimes();
    Timing::printTimeSums();

    return 0;
}

void reset()
{
    Timing::printAverageTimes();
    Timing::reset();

    base.getSimulationMethod().simulation->reset();
    TimeManager::getCurrent()->setTime(0.0);
}


void timeStep()
{
    if((base.getPauseAt() > 0.0) && (base.getPauseAt() < TimeManager::getCurrent()->getTime()))
        base.setPause(true);

    if(base.getPause())
        return;

    // Simulation code
    for(unsigned int i = 0; i < base.getNumberOfStepsPerRenderUpdate(); i++)
    {
        START_TIMING("SimStep");
        base.getSimulationMethod().simulation->step();
        STOP_TIMING_AVG;

        base.getSimulationMethod().model.writeFrameFluidData(TimeManager::getCurrent()->getTime());
    }
}


void render()
{
    MiniGL::coordinateSystem();

    base.renderFluid();
    renderBoundary();
}

void renderBoundary()
{
    DemoBase::SimulationMethod& simulationMethod      = base.getSimulationMethod();
    Shader&                     shader                = base.getShader();
    Shader&                     meshShader            = base.getMeshShader();
    SceneLoader::Scene&         scene                 = base.getScene();
    const int                   renderWalls           = base.getRenderWalls();
    GLint                       context_major_version = base.getContextMajorVersion();

    float                       wallColor[4] = { 0.1f, 0.6f, 0.6f, 1.0f };
    if((renderWalls == 1) || (renderWalls == 2))
    {
        if(context_major_version > 3)
        {
            shader.begin();
            glUniform3fv(shader.getUniform("color"), 1, &wallColor[0]);
            glEnableVertexAttribArray(0);
            for(int body = simulationMethod.model.numberOfRigidBodyParticleObjects() - 1; body >= 0; body--)
            {
                if((renderWalls == 1) || (!scene.boundaryModels[body]->isWall))
                {
                    FluidModel::RigidBodyParticleObject* rb = simulationMethod.model.getRigidBodyParticleObject(body);
                    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, &simulationMethod.model.getPosition(body + 1, 0));
                    glDrawArrays(GL_POINTS, 0, rb->numberOfParticles());
                }
            }
            glDisableVertexAttribArray(0);
            shader.end();
        }
        else
        {
            glDisable(GL_LIGHTING);
            glPointSize(4.0f);

            glBegin(GL_POINTS);
            for(int body = simulationMethod.model.numberOfRigidBodyParticleObjects() - 1; body >= 0; body--)
            {
                if((renderWalls == 1) || (!scene.boundaryModels[body]->isWall))
                {
                    FluidModel::RigidBodyParticleObject* rb = simulationMethod.model.getRigidBodyParticleObject(body);
                    for(unsigned int i = 0; i < rb->numberOfParticles(); i++)
                    {
                        glColor3fv(wallColor);
                        glVertex3v(&simulationMethod.model.getPosition(body + 1, i)[0]);
                    }
                }
            }
            glEnd();
            glEnable(GL_LIGHTING);
        }
    }
    else if((renderWalls == 3) || (renderWalls == 4))
    {
        for(int body = simulationMethod.model.numberOfRigidBodyParticleObjects() - 1; body >= 0; body--)
        {
            if((renderWalls == 3) || (!scene.boundaryModels[body]->isWall))
            {
                meshShader.begin();
                glUniform1f(meshShader.getUniform("shininess"), 5.0f);
                glUniform1f(meshShader.getUniform("specular_factor"), 0.2f);

                GLfloat matrix[16];
                glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
                glUniformMatrix4fv(meshShader.getUniform("modelview_matrix"), 1, GL_FALSE, matrix);
                GLfloat pmatrix[16];
                glGetFloatv(GL_PROJECTION_MATRIX, pmatrix);
                glUniformMatrix4fv(meshShader.getUniform("projection_matrix"), 1, GL_FALSE, pmatrix);

                glUniform3fv(meshShader.getUniform("surface_color"), 1, wallColor);

                FluidModel::RigidBodyParticleObject* rb = simulationMethod.model.getRigidBodyParticleObject(body);
                MiniGL::drawMesh(((StaticRigidBody*)rb->m_rigidBody)->getGeometry(), wallColor);

                meshShader.end();
            }
        }
    }
}



void initBoundaryData()
{
    std::string         base_path = FileSystem::getFilePath(base.getSceneFile());
    SceneLoader::Scene& scene     = base.getScene();
    const bool          useCache  = base.getUseParticleCaching();

    for(unsigned int i = 0; i < scene.boundaryModels.size(); i++)
    {
        string                meshFileName = FileSystem::normalizePath(base_path + "/" + scene.boundaryModels[i]->meshFile);

        std::vector<Vector3r> boundaryParticles;
        if(scene.boundaryModels[i]->samplesFile != "")
        {
            string particleFileName = base_path + "/" + scene.boundaryModels[i]->samplesFile;
            PartioReaderWriter::readParticles(particleFileName, scene.boundaryModels[i]->translation, scene.boundaryModels[i]->rotation, scene.boundaryModels[i]->scale[0], boundaryParticles);
        }

        // Cache sampling
        std::string mesh_base_path   = FileSystem::getFilePath(scene.boundaryModels[i]->meshFile);
        std::string mesh_file_name   = FileSystem::getFileName(scene.boundaryModels[i]->meshFile);
        std::string scene_path       = FileSystem::getFilePath(base.getSceneFile());
        std::string scene_file_name  = FileSystem::getFileName(base.getSceneFile());
        string      cachePath        = scene_path + "/" + mesh_base_path + "/Cache";
        string      particleFileName = FileSystem::normalizePath(cachePath + "/" + scene_file_name + "_" + mesh_file_name + "_" + std::to_string(i) + ".bgeo");


        StaticRigidBody* rb  = new StaticRigidBody();
        TriangleMesh&    geo = rb->getGeometry();
        OBJLoader::loadObj(meshFileName, geo, scene.boundaryModels[i]->scale);
        for(unsigned int j = 0; j < geo.numVertices(); j++)
            geo.getVertices()[j] = scene.boundaryModels[i]->rotation * geo.getVertices()[j] + scene.boundaryModels[i]->translation;

        geo.updateNormals();
        geo.updateVertexNormals();

        if(scene.boundaryModels[i]->samplesFile == "")
        {
            bool foundCacheFile = false;
            if(useCache)
            {
                foundCacheFile = PartioReaderWriter::readParticles(particleFileName, Vector3r::Zero(), Matrix3r::Identity(), 1.0, boundaryParticles);
                if(foundCacheFile)
                    std::cout << "Loaded cached boundary sampling: " << particleFileName << "\n";
            }

            if(!useCache || !foundCacheFile)
            {
                std::cout << "Surface sampling of " << meshFileName << "\n";
                START_TIMING("Poisson disk sampling");
                PoissonDiskSampling sampling;
                sampling.sampleMesh(geo.numVertices(), geo.getVertices().data(), geo.numFaces(), geo.getFaces().data(), scene.particleRadius, 10, 1, boundaryParticles);
                STOP_TIMING_AVG;

                // Cache sampling
                if(useCache && (FileSystem::makeDir(cachePath) == 0))
                {
                    std::cout << "Save particle sampling: " << particleFileName << "\n";
                    PartioReaderWriter::writeParticles(particleFileName, (unsigned int)boundaryParticles.size(), boundaryParticles.data(), NULL, scene.particleRadius);
                }
            }
        }


        base.getSimulationMethod().model.addRigidBodyObject(rb, static_cast<unsigned int>(boundaryParticles.size()), &boundaryParticles[0]);
    }
}

