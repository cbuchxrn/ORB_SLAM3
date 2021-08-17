/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#ifndef ATLAS_H
#define ATLAS_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

#include <set>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>


namespace ORB_SLAM3
{
class Viewer;
class Map;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class Frame;
class KannalaBrandt8;
class Pinhole;
class System;
class Atlas
{
//Singelton Pattern
//########################################
public:
    static Atlas* getInstance();
    Atlas(Atlas const&)           = delete;
    void operator=(Atlas const&)  = delete;
private:
    Atlas();
    Atlas(int initKFid); // When its initialization the first map is created
//########################################
public:
    ~Atlas();
    void registerSys(ORB_SLAM3::System* currentSystem);
    void CreateNewMap();
    void CreateNewMap(int SysID);
    void ChangeMap(int SysID,Map* pMap);

    unsigned long int GetLastInitKFid();

    void SetViewer(Viewer* pViewer);

    // Method for change components in the current map
    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);

    void AddCamera(GeometricCamera* pCam);

    /* All methods without Map pointer work on current map */
    void SetReferenceMapPoints(int SysID,const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange(int SysID);
    int GetLastBigChangeIdx(int SysID);

    long unsigned int MapPointsInMap(int SysID);
    long unsigned KeyFramesInMap(int SysID);

    // Method for get data in current map
    std::vector<KeyFrame*> GetAllKeyFrames(int SysID);
    std::vector<MapPoint*> GetAllMapPoints(int SysID);
    std::vector<MapPoint*> GetReferenceMapPoints(int SysID);

    vector<Map*> GetAllMaps();

    int CountMaps();

    void clearMap(int SysID);

    void clearAtlas(int SysID);

    Map* GetCurrentMap(int SysID);

    void SetMapBad(Map* pMap);
    void RemoveBadMaps();

    bool isInertial(int SysID);
    void SetInertialSensor(int SysID);
    void SetImuInitialized(int SysID);
    bool isImuInitialized(int SysID);

    void SetKeyFrameDababase(KeyFrameDatabase* pKFDB);
    KeyFrameDatabase* GetKeyFrameDatabase();

    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    ORBVocabulary* GetORBVocabulary();

    long unsigned int GetNumLivedKF();

    long unsigned int GetNumLivedMP();

protected:

    std::set<Map*> mspMaps;
    std::set<Map*> mspBadMaps;
    std::vector<Map*> mpCurrentMap;

    std::vector<ORB_SLAM3::System*> mvpRegisteredSystems;
    
    std::vector<GeometricCamera*> mvpCameras;
    std::vector<KannalaBrandt8*> mvpBackupCamKan;
    std::vector<Pinhole*> mvpBackupCamPin;

    std::mutex mMutexAtlas;

    unsigned long int mnLastInitKFidMap;

    Viewer* mpViewer;
    bool mHasViewer;

    // Class references for the map reconstruction from the save file
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;


}; // class Atlas

} // namespace ORB_SLAM3

#endif // ATLAS_H
