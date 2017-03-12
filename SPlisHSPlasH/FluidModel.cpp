#include "FluidModel.h"
#include "SPHKernels.h"
#include <iostream>

using namespace SPH;

FluidModel::FluidModel()
{
    m_density0               = 1000.0;
    m_particleRadius         = 0.025;
    m_viscosity              = 0.02;
    m_neighborhoodSearch     = NULL;
    m_gravitation            = Vector3r(0.0, -9.81, 0.0);
    m_stiffness              = 50000.0;
    m_exponent               = 7.0;
    m_surfaceTension         = 0.05;
    m_enableDivergenceSolver = true;
    m_velocityUpdateMethod   = 0;

    ParticleObject* fluidParticles = new ParticleObject();
    m_particleObjects.push_back(fluidParticles);

    setKernel(0);
    setGradKernel(0);
}

FluidModel::~FluidModel(void)
{
    cleanupModel();
}

void FluidModel::cleanupModel()
{
    releaseFluidParticles();
    for(unsigned int i = 0; i < m_particleObjects.size(); i++)
    {
        if(i > 0)
        {
            RigidBodyParticleObject* rbpo = ((RigidBodyParticleObject*)m_particleObjects[i]);
            rbpo->m_boundaryPsi.clear();
            rbpo->m_f.clear();
            delete rbpo->m_rigidBody;
            delete rbpo;
        }
        else
            delete m_particleObjects[i];
    }
    m_particleObjects.clear();

    m_a.clear();
    m_masses.clear();
    m_density.clear();
    delete m_neighborhoodSearch;
}

void FluidModel::reset()
{
    const unsigned int nPoints = numParticles();

    // reset velocities and accelerations
    for(unsigned int i = 1; i < m_particleObjects.size(); i++)
    {
        for(int j = 0; j < (int)m_particleObjects[i]->m_x.size(); j++)
        {
            RigidBodyParticleObject* rbpo = ((RigidBodyParticleObject*)m_particleObjects[i]);
            rbpo->m_f[j].setZero();
            rbpo->m_v[j].setZero();
        }
    }

    // Fluid
    for(unsigned int i = 0; i < nPoints; i++)
    {
        const Vector3r& x0 = getPosition0(0, i);
        getPosition(0, i) = x0;
        getVelocity(0, i).setZero();
        getAcceleration(i).setZero();
        m_density[i] = 0.0;
    }

    if(m_neighborhoodSearch != NULL)
    {
        performNeighborhoodSearchSort();
    }
    updateBoundaryPsi();
}

void FluidModel::initMasses()
{
    const int  nParticles = (int)numParticles();
    const Real diam       = 2.0 * m_particleRadius;

#pragma omp parallel default(shared)
    {
#pragma omp for schedule(static)
        for(int i = 0; i < nParticles; i++)
        {
            setMass(i, 0.8 * diam * diam * diam * m_density0);              // each particle represents a cube with a side length of r
                                                                            // mass is slightly reduced to prevent pressure at the beginning of the simulation
        }
    }
}

void FluidModel::resizeFluidParticles(const unsigned int newSize)
{
    m_particleObjects[0]->m_x0.resize(newSize);
    m_particleObjects[0]->m_x.resize(newSize);
    m_particleObjects[0]->m_v.resize(newSize);
    m_a.resize(newSize);
    m_masses.resize(newSize);
    m_density.resize(newSize);
}

void FluidModel::releaseFluidParticles()
{
    m_particleObjects[0]->m_x0.clear();
    m_particleObjects[0]->m_x.clear();
    m_particleObjects[0]->m_v.clear();
    m_a.clear();
    m_masses.clear();
    m_density.clear();
}

void FluidModel::initModel(const unsigned int nFluidParticles, Vector3r* fluidParticles)
{
    releaseFluidParticles();
    resizeFluidParticles(nFluidParticles);

    // init kernel
    setParticleRadius(m_particleRadius);

    // copy fluid positions
#pragma omp parallel default(shared)
    {
#pragma omp for schedule(static)
        for(int i = 0; i < (int)nFluidParticles; i++)
        {
            getPosition0(0, i) = fluidParticles[i];
        }
    }

    // initialize masses
    initMasses();

    // Initialize neighborhood search
    if(m_neighborhoodSearch == NULL)
        m_neighborhoodSearch = new CompactNSearch::NeighborhoodSearch(m_supportRadius, false);
    m_neighborhoodSearch->set_radius(m_supportRadius);

    // Fluids
    m_neighborhoodSearch->add_point_set(&getPosition(0, 0)[0], nFluidParticles, true, true);

    // Boundary
    for(unsigned int i = 0; i < numberOfRigidBodyParticleObjects(); i++)
    {
        RigidBodyParticleObject* rb = getRigidBodyParticleObject(i);
        m_neighborhoodSearch->add_point_set(&rb->m_x[0][0], rb->m_x.size(), rb->m_rigidBody->isDynamic(), false);
    }

    reset();
}

void FluidModel::updateBoundaryPsi()
{
    //////////////////////////////////////////////////////////////////////////
    // Compute value psi for boundary particles (boundary handling)
    // (see Akinci et al. "Versatile rigid - fluid coupling for incompressible SPH", Siggraph 2012
    //////////////////////////////////////////////////////////////////////////

    // Search boundary neighborhood

    // Activate only static boundaries
    std::cout << "Initialize boundary psi\n";
    m_neighborhoodSearch->point_set(0).enable_neighborsearch(false);
    for(unsigned int i = 0; i < numberOfRigidBodyParticleObjects(); i++)
    {
        if(!getRigidBodyParticleObject(i)->m_rigidBody->isDynamic())
            m_neighborhoodSearch->point_set(i + 1).enable_neighborsearch(true);
    }

    m_neighborhoodSearch->find_neighbors();

    // Boundary objects
    for(unsigned int body = 0; body < numberOfRigidBodyParticleObjects(); body++)
    {
        if(!getRigidBodyParticleObject(body)->m_rigidBody->isDynamic())
            computeBoundaryPsi(body);
    }

    //////////////////////////////////////////////////////////////////////////
    // Compute boundary psi for all dynamic bodies
    //////////////////////////////////////////////////////////////////////////
    for(unsigned int body = 0; body < numberOfRigidBodyParticleObjects(); body++)
    {
        // Deactivate all
        for(int j = 1; j < m_neighborhoodSearch->point_sets().size(); j++)
            m_neighborhoodSearch->point_set(j).enable_neighborsearch(false);

        // Only activate next dynamic body
        if(getRigidBodyParticleObject(body)->m_rigidBody->isDynamic())
        {
            m_neighborhoodSearch->point_set(body + 1).enable_neighborsearch(true);
            m_neighborhoodSearch->find_neighbors();
            computeBoundaryPsi(body);
        }
    }

    // Activate only fluids
    m_neighborhoodSearch->point_set(0).enable_neighborsearch(true);
    for(int i = 1; i < m_neighborhoodSearch->point_sets().size(); i++)
        m_neighborhoodSearch->point_set(i).enable_neighborsearch(false);
}

void FluidModel::computeBoundaryPsi(const unsigned int body)
{
    const Real               density0 = getDensity0();

    RigidBodyParticleObject* rb                   = getRigidBodyParticleObject(body);
    const unsigned int       numBoundaryParticles = rb->numberOfParticles();

#pragma omp parallel default(shared)
    {
#pragma omp for schedule(static)
        for(int i = 0; i < (int)numBoundaryParticles; i++)
        {
            Real delta = m_W_zero;
            for(unsigned int j = 0; j < m_neighborhoodSearch->point_set(body + 1).n_neighbors(i); j++)
            {
                const CompactNSearch::PointID& pid = m_neighborhoodSearch->point_set(body + 1).neighbor(i, j);
                if(pid.point_set_id != 0)
                    delta += W(getPosition(body + 1, i) - getPosition(pid.point_set_id, pid.point_id));
            }
            const Real volume = 1.0 / delta;
            rb->m_boundaryPsi[i] = density0 * volume;
        }
    }
}

void FluidModel::addRigidBodyObject(RigidBodyObject* rbo, const unsigned int numBoundaryParticles, Vector3r* boundaryParticles)
{
    RigidBodyParticleObject* rb = new RigidBodyParticleObject();
    m_particleObjects.push_back(rb);

    rb->m_x0.resize(numBoundaryParticles);
    rb->m_x.resize(numBoundaryParticles);
    rb->m_v.resize(numBoundaryParticles);
    rb->m_f.resize(numBoundaryParticles);
    rb->m_boundaryPsi.resize(numBoundaryParticles);

#pragma omp parallel default(shared)
    {
#pragma omp for schedule(static)
        for(int i = 0; i < (int)numBoundaryParticles; i++)
        {
            rb->m_x0[i] = boundaryParticles[i];
            rb->m_x[i]  = boundaryParticles[i];
            rb->m_v[i].setZero();
            rb->m_f[i].setZero();
        }
    }
    rb->m_rigidBody = rbo;
}

void FluidModel::performNeighborhoodSearchSort()
{
    const unsigned int numPart = numParticles();
    if(numPart == 0)
        return;

    m_neighborhoodSearch->z_sort();

    auto const& d = m_neighborhoodSearch->point_set(0);
    d.sort_field(&m_particleObjects[0]->m_x0[0]);
    d.sort_field(&m_particleObjects[0]->m_x[0]);
    d.sort_field(&m_particleObjects[0]->m_v[0]);
    d.sort_field(&m_a[0]);
    d.sort_field(&m_masses[0]);
    d.sort_field(&m_density[0]);


    //////////////////////////////////////////////////////////////////////////
    // Boundary
    //////////////////////////////////////////////////////////////////////////
    for(unsigned int i = 1; i < m_neighborhoodSearch->point_sets().size(); i++)
    {
        RigidBodyParticleObject* rb = getRigidBodyParticleObject(i - 1);
        if(rb->m_rigidBody->isDynamic())                        // sort only dynamic boundaries
        {
            auto const& d = m_neighborhoodSearch->point_set(i);
            d.sort_field(&rb->m_x0[0]);
            d.sort_field(&rb->m_x[0]);
            d.sort_field(&rb->m_v[0]);
            d.sort_field(&rb->m_f[0]);
            d.sort_field(&rb->m_boundaryPsi[0]);
        }
    }
}

void SPH::FluidModel::setParticleRadius(Real val)
{
    m_particleRadius = val;
    m_supportRadius  = 4.0 * m_particleRadius;

    // init kernel
    Poly6Kernel::setRadius(m_supportRadius);
    SpikyKernel::setRadius(m_supportRadius);
    CubicKernel::setRadius(m_supportRadius);
    PrecomputedCubicKernel::setRadius(m_supportRadius);
    CohesionKernel::setRadius(m_supportRadius);
    AdhesionKernel::setRadius(m_supportRadius);
}


void SPH::FluidModel::setGradKernel(unsigned int val)
{
    m_gradKernelMethod = val;
    if(m_gradKernelMethod == 0)
        m_gradKernelFct = CubicKernel::gradW;
    else if(m_gradKernelMethod == 1)
        m_gradKernelFct = Poly6Kernel::gradW;
    else if(m_gradKernelMethod == 2)
        m_gradKernelFct = SpikyKernel::gradW;
    else if(m_gradKernelMethod == 3)
        m_gradKernelFct = FluidModel::PrecomputedCubicKernel::gradW;
}

void SPH::FluidModel::setKernel(unsigned int val)
{
    m_kernelMethod = val;
    if(m_kernelMethod == 0)
    {
        m_W_zero    = CubicKernel::W_zero();
        m_kernelFct = CubicKernel::W;
    }
    else if(m_kernelMethod == 1)
    {
        m_W_zero    = Poly6Kernel::W_zero();
        m_kernelFct = Poly6Kernel::W;
    }
    else if(m_kernelMethod == 2)
    {
        m_W_zero    = SpikyKernel::W_zero();
        m_kernelFct = SpikyKernel::W;
    }
    else if(m_kernelMethod == 3)
    {
        m_W_zero    = FluidModel::PrecomputedCubicKernel::W_zero();
        m_kernelFct = FluidModel::PrecomputedCubicKernel::W;
    }
}


//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
#define AniGen_Lambda                 0.1
#define AniGen_NeighborCountThreshold 25
#define AniGen_Kr                     4

void SPH::FluidModel::generateAniKernels(std::vector<Vector3r>& kernelCenters, std::vector<Matrix3r>& kernelMatrices)
{
    static Real aniKernelRadius    = 8.0 * m_particleRadius;
    static Real aniKernelRadiusInv = 1.0 / aniKernelRadius;
    static Real aniKernelRadiusSqr = aniKernelRadius * aniKernelRadius;
    const int   numFluidParticles  = (int)numParticles();
    static auto kernelW = [] (Real d, Real aniKernelRadiusInv)
                          {
                              return 1.0 - pow(d * aniKernelRadiusInv, 3);
                          };

    kernelCenters.resize(m_particleObjects[0]->m_x.size());
    kernelMatrices.resize(m_particleObjects[0]->m_x.size());

    static CompactNSearch::NeighborhoodSearch* aniNeighborhoodSearch = nullptr;

    if(aniNeighborhoodSearch == nullptr)
    {
        aniNeighborhoodSearch = new CompactNSearch::NeighborhoodSearch(m_supportRadius, false);
        aniNeighborhoodSearch->set_radius(aniKernelRadius);
        aniNeighborhoodSearch->add_point_set(&getPosition(0, 0)[0], m_particleObjects[0]->m_x.size(), true, true);
    }

    aniNeighborhoodSearch->find_neighbors();


#pragma omp parallel default(shared)
    {
#pragma omp for schedule(static)
        for(int i = 0; i < numFluidParticles; i++)
        {
            const Vector3r& xi = getPosition(0, i);

            Vector3r        pposWM       = xi;
            Real            sumW         = 1.0;
            unsigned int    numNeighbors = static_cast<unsigned int>(aniNeighborhoodSearch->point_set(0).n_neighbors(i));

            for(unsigned int j = 0; j < numNeighbors; j++)
            {
                const CompactNSearch::PointID& particleId = aniNeighborhoodSearch->point_set(0).neighbor(i, j);
                const Vector3r&                xj         = getPosition(0, particleId.point_id);

                const Vector3r                 xij = xj - xi;
                const Real                     d2  = xij.squaredNorm();
                if(d2 < aniKernelRadiusSqr)
                {
                    const Real wij = kernelW(sqrt(d2), aniKernelRadiusInv);
                    sumW   += wij;
                    pposWM += wij * xj;
                }
            }

            assert(sumW > 0);
            pposWM          /= sumW;
            kernelCenters[i] = (1.0 - AniGen_Lambda) * xi + AniGen_Lambda * pposWM;


            ////////////////////////////////////////////////////////////////////////////////
            // compute covariance matrix and anisotropy matrix
            unsigned int neighborCount = 0;
            Matrix3r     C             = (xi - pposWM) * (xi - pposWM).transpose();
            sumW = 1.0;

            for(unsigned int j = 0; j < numNeighbors; j++)
            {
                const CompactNSearch::PointID& particleId = aniNeighborhoodSearch->point_set(0).neighbor(i, j);
                const Vector3r&                xj         = getPosition(0, particleId.point_id);

                const Vector3r                 xij = xj - pposWM;
                const Real                     d2  = xij.squaredNorm();
                if(d2 < aniKernelRadiusSqr)
                {
                    const Real wij = kernelW(sqrt(d2), aniKernelRadiusInv);
                    sumW += wij;

                    C += wij * xij * xij.transpose();

                    ++neighborCount;
                }
            }

            assert(sumW > 0);
            C /= sumW; // = covariance matrix

            ////////////////////////////////////////////////////////////////////////////////
            // compute kernel matrix
            Matrix3r U, S, V;
            SVDDecomposition::svd(C(0, 0), C(0, 1), C(0, 2), C(1, 0), C(1, 1), C(1, 2), C(2, 0), C(2, 1), C(2, 2),
                U(0, 0), U(0, 1), U(0, 2), U(1, 0), U(1, 1), U(1, 2), U(2, 0), U(2, 1), U(2, 2),
                S(0, 0), S(0, 1), S(0, 2), S(1, 0), S(1, 1), S(1, 2), S(2, 0), S(2, 1), S(2, 2),
                V(0, 0), V(0, 1), V(0, 2), V(1, 0), V(1, 1), V(1, 2), V(2, 0), V(2, 1), V(2, 2));

            Vector3r sigmas = static_cast<Real>(0.75) * Vector3r(1, 1, 1);

            if(neighborCount > AniGen_NeighborCountThreshold)
            {
                sigmas = Vector3r(S(0, 0), std::max(S(1, 1), S(0, 0) / AniGen_Kr), std::max(S(2, 2), S(0, 0) / AniGen_Kr));
                Real ks = std::cbrt(1.0 / (sigmas[0] * sigmas[1] * sigmas[2]));          // scale so that det(covariance) == 1
                sigmas *= ks;
            }

            S << sigmas[0], 0.0, 0.0,
                0.0, sigmas[1], 0.0,
                0.0, 0.0, sigmas[2];
            kernelMatrices[i] = U * S * U.transpose();
        }
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
#include "SPlisHSPlasH/StaticRigidBody.h"

int SPH::FluidModel::writeFrameFluidData(Real currentTime)
{
    static double savedFrameTime = -1000000000.0;
    static int    frame          = 0;

    if(currentTime < savedFrameTime + m_FrameTime)
    {
        return -1;
    }

    savedFrameTime += m_FrameTime;
    ++frame;

    // for saving frame the first time
    if(savedFrameTime < 0)
        savedFrameTime = currentTime;


    ////////////////////////////////////////////////////////////////////////////////
    if(m_FluidPosWriter == nullptr)
    {
        m_FluidPosWriter = new DataIO(m_SaveDataPath, "FluidFrame", "frame", "pos");
    }

    if(m_FluidVelWriter == nullptr)
    {
        m_FluidVelWriter = new DataIO(m_SaveDataPath, "FluidFrame", "frame", "vel");
    }

    if(m_FluidAnisotropyWriter == nullptr)
    {
        m_FluidAnisotropyWriter = new DataIO(m_SaveDataPath, "FluidFrame", "frame", "ani");
    }


    ////////////////////////////////////////////////////////////////////////////////
    static std::vector<Vector3r> kernelCenters;
    static std::vector<Matrix3r> kernelMatrices;
    generateAniKernels(kernelCenters, kernelMatrices);

    m_FluidPosWriter->reset_buffer();
    m_FluidPosWriter->getBuffer().push_back(static_cast<unsigned int>(numParticles()));
    m_FluidPosWriter->getBuffer().push_back_to_float(m_particleRadius);
    m_FluidPosWriter->getBuffer().push_back_to_float_array(kernelCenters, false);
    m_FluidPosWriter->flush_buffer_async(frame);

    m_FluidVelWriter->reset_buffer();
    m_FluidVelWriter->getBuffer().push_back_to_float_array(m_particleObjects[0]->m_v);
    m_FluidVelWriter->flush_buffer_async(frame);


    m_FluidAnisotropyWriter->reset_buffer();
    m_FluidAnisotropyWriter->getBuffer().push_back_to_float_array(kernelMatrices);
    m_FluidAnisotropyWriter->flush_buffer_async(frame);

    ////////////////////////////////////////////////////////////////////////////////
    return frame;
}
