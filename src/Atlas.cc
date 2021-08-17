/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Atlas.h"
#include "Viewer.h"

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace ORB_SLAM3
{

void Atlas::registerSys(ORB_SLAM3::System* currentSystem)
{
    // retruns id for later references
    int index = 0;
    for(ORB_SLAM3::System* pSystem : this->mvpRegisteredSystems)
    {   
        if (currentSystem == pSystem)
        {
            currentSystem->setSysId(index);
        }
        index ++;
    }


    // If the element is not
    // present in the vector
    this->mvpRegisteredSystems.push_back(currentSystem);
    this->mpCurrentMap.push_back(static_cast<Map*>(NULL));
    CreateNewMap(index);
    currentSystem->setSysId(index);
    
}

Atlas* Atlas::getInstance()
{
    static Atlas* instance = new Atlas(0);
    return instance;
}

Atlas::Atlas(){
    
}

Atlas::Atlas(int initKFid): mnLastInitKFidMap(initKFid), mHasViewer(false)
{

}

Atlas::~Atlas()
{
    for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        Map* pMi = *it;

        if(pMi)
        {
            delete pMi;
            pMi = static_cast<Map*>(NULL);

            it = mspMaps.erase(it);
        }
        else
            ++it;

    }
}

void Atlas::CreateNewMap()
{
    // create a new map for each connected slam system 
    int nSys = mvpRegisteredSystems.size();
    for(int iSys = 0;iSys < nSys;iSys++) 
    {
        Atlas::CreateNewMap(iSys);
    }

}

void Atlas::CreateNewMap(int SysID)
{
    // create a new map for slam system with given id 

    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Creation of new map with id: " << Map::nNextId << endl;
    if(mpCurrentMap[SysID]){
        cout << "Exits current map " << endl;
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap[SysID]->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap[SysID]->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap[SysID]->SetStoredMap();
        cout << "Saved map with ID: " << mpCurrentMap[SysID]->GetId() << endl;

        //if(mHasViewer)
        //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

    mpCurrentMap[SysID] = new Map(mnLastInitKFidMap);
    mpCurrentMap[SysID]->SetCurrentMap();
    mspMaps.insert(mpCurrentMap[SysID]);
}

void Atlas::ChangeMap(int SysID, Map* pMap)
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Chage to map with id: " << pMap->GetId() << endl;
    if(mpCurrentMap[SysID]){
        mpCurrentMap[SysID]->SetStoredMap();
    }

    mpCurrentMap[SysID] = pMap;
    mpCurrentMap[SysID]->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer* pViewer)
{
    mpViewer = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame* pKF)
{
    Map* pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint* pMP)
{
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
{
    //Check if the camera already exists
    bool bAlreadyInMap = false;
    int index_cam = -1;
    for(size_t i=0; i < mvpCameras.size(); ++i)
    {
        GeometricCamera* pCam_i = mvpCameras[i];
        if(!pCam) std::cout << "Not pCam" << std::endl;
        if(!pCam_i) std::cout << "Not pCam_i" << std::endl;
        if(pCam->GetType() != pCam_i->GetType())
            continue;

        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            if(((Pinhole*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
        else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            if(((KannalaBrandt8*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
    }

    if(bAlreadyInMap)
    {
        return mvpCameras[index_cam];
    }
    else{
        mvpCameras.push_back(pCam);
        return pCam;
    }
}

std::vector<GeometricCamera*> Atlas::GetAllCameras()
{
    return mvpCameras;
}

void Atlas::SetReferenceMapPoints(int SysID,const std::vector<MapPoint*> &vpMPs)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap[SysID]->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap[SysID]->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap[SysID]->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap[SysID]->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap[SysID]->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap[SysID]->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap[SysID]->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap[SysID]->GetReferenceMapPoints();
}

vector<Map*> Atlas::GetAllMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector<Map*> vMaps(mspMaps.begin(),mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

int Atlas::CountMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap[SysID]->clear();
}

void Atlas::clearAtlas(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    mpCurrentMap[SysID] = static_cast<Map*>(NULL);
    mnLastInitKFidMap = 0;
}

Map* Atlas::GetCurrentMap(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(!mpCurrentMap[SysID])
        CreateNewMap();
    while(mpCurrentMap[SysID]->IsBad())
        usleep(3000);

    return mpCurrentMap[SysID];
}

void Atlas::SetMapBad(Map* pMap)
{
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps()
{
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(NULL);
    }*/
    mspBadMaps.clear();
}

bool Atlas::isInertial(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap[SysID]->IsInertial();
}

void Atlas::SetInertialSensor(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap[SysID]->SetInertialSensor();
}

void Atlas::SetImuInitialized(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap[SysID]->SetImuInitialized();
}

bool Atlas::isImuInitialized(int SysID)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap[SysID]->isImuInitialized();
}

void Atlas::PreSave()
{
    if(mpCurrentMap){
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum
    }

    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
    sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

    std::set<GeometricCamera*> spCams(mvpCameras.begin(), mvpCameras.end());
    for(Map* pMi : mvpBackupMaps)
    {
        if(!pMi || pMi->IsBad())
            continue;

        if(pMi->GetAllKeyFrames().size() == 0) {
            // Empty map, erase before of save it.
            SetMapBad(pMi);
            continue;
        }
        pMi->PreSave(spCams);
    }
    RemoveBadMaps();
}

void Atlas::PostLoad()
{
    map<unsigned int,GeometricCamera*> mpCams;
    for(GeometricCamera* pCam : mvpCameras)
    {
        mpCams[pCam->GetId()] = pCam;
    }

    mspMaps.clear();
    unsigned long int numKF = 0, numMP = 0;
    for(Map* pMi : mvpBackupMaps)
    {
        mspMaps.insert(pMi);
        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }
    mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map* pMap_i : mspMaps)
    {
        num += pMap_i->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP() {
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map* pMap_i : mspMaps) {
        num += pMap_i->GetAllMapPoints().size();
    }

    return num;
}

map<long unsigned int, KeyFrame*> Atlas::GetAtlasKeyframes()
{
    map<long unsigned int, KeyFrame*> mpIdKFs;
    for(Map* pMap_i : mvpBackupMaps)
    {
        vector<KeyFrame*> vpKFs_Mi = pMap_i->GetAllKeyFrames();

        for(KeyFrame* pKF_j_Mi : vpKFs_Mi)
        {
            mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
        }
    }

    return mpIdKFs;
}

} //namespace ORB_SLAM3
