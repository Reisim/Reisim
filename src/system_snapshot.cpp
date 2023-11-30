#include "systemthread.h"


void SystemThread::OutputRestartData(QString filename)
{
    QFile file(filename);
    if( file.open( QIODevice::WriteOnly | QIODevice::Text ) == false ){
        qDebug() << "Cannot open file: " << filename;
        return;
    }

    QTextStream out( &file );

    out << "#-------------------------------------------------------\n";
    out << "#                 Re:sim Restart Data                   \n";
    out << "#-------------------------------------------------------\n";
    out << "\n";


    out << "Time-Stamp:" << simManage->GetSimulationTimeStr() << "\n";
    out << "\n";


    for(int i=0;i<maxAgent;++i){

        if( agent[i]->agentStatus == 0 ){
            continue;
        }

        if( agent[i]->isScenarioObject == true ){
            continue;
        }

        if( agent[i]->isSInterfaceObject == true ){
            continue;
        }


        out << "A," << i << ","
            << agent[i]->ID << ","
            << agent[i]->agentKind << ","
            << agent[i]->isScenarioObject << ","
            << agent[i]->isBehaviorEmbeded << ","
            << agent[i]->isSInterfaceObject << ","
            << agent[i]->notAllowedAppear << ","
            << agent[i]->calInterval << ","
            << agent[i]->cognitionCountMax << ","
            << agent[i]->cognitionCount << ","
            << agent[i]->decisionMakingCountMax << ","
            << agent[i]->decisionMakingCount << ","
            << agent[i]->controlCountMax << ","
            << agent[i]->controlCount << "\n";


        out << agent[i]->attri.age << ","
            << agent[i]->attri.gender << "\n";


        out << agent[i]->param.startRelay << ","
            << agent[i]->param.headwayTime << ","
            << agent[i]->param.latAccelAtTurn << ","
            << agent[i]->param.maxDeceleration << ","
            << agent[i]->param.visibleDistance << ","
            << agent[i]->param.accelControlGain << ","
            << agent[i]->param.safetyConfirmTime << ","
            << agent[i]->param.headwayControlGain << ","
            << agent[i]->param.steeringControlGain << ","
            << agent[i]->param.accelOffDeceleration << ","
            << agent[i]->param.deadZoneSpeedControl << ","
            << agent[i]->param.crossTimeSafetyMargin << ","
            << agent[i]->param.minimumDistanceToStopLine << ","
            << agent[i]->param.crossWaitPositionSafeyMargin << ","
            << agent[i]->param.minimumHeadwayDistanceAtStop << ","
            << agent[i]->param.pedestWaitPositionSafetyMargin << ","
            << agent[i]->param.minimumPerceptibleDecelerationOfPreceding << ","
            << agent[i]->param.speedVariationFactor << "\n";

        if( agent[i]->agentKind < 100 ){

            out << agent[i]->vehicle.state.X << ","
                << agent[i]->vehicle.state.Y << ","
                << agent[i]->vehicle.state.Z << ","
                << agent[i]->vehicle.state.vx << ","
                << agent[i]->vehicle.state.yawAngle << ","
                << agent[i]->vehicle.state.roll << ","
                << agent[i]->vehicle.state.pitch << ","
                << agent[i]->vehicle.state.yawRate << ","
                << agent[i]->vehicle.state.ax << ","
                << agent[i]->state.accel << ","
                << agent[i]->state.brake << ","
                << agent[i]->state.steer << ","
                << agent[i]->vehicle.winker_state << ","
                << agent[i]->vehicle.winker_count << ","
                << agent[i]->vehicle.vehicleModelID << "\n";

            out << agent[i]->memory.routeType << ","
                << agent[i]->memory.routeIndex << ","
                << agent[i]->memory.routeLaneIndex << ","
                << agent[i]->memory.currentTargetPath << ","
                << agent[i]->memory.currentTargetPathIndexInList << ","
                << agent[i]->memory.distanceFromStartWPInCurrentPath << ","
                << agent[i]->memory.scenarioPathSelectID << ","
                << agent[i]->memory.distanceToTurnNodeWPIn << ","
                << agent[i]->memory.distanceToNodeWPIn << ","
                << agent[i]->memory.distanceToTurnNodeWPOut << ","
                << agent[i]->memory.distanceToNodeWPOut << ","
                << agent[i]->memory.LCSupportRouteLaneIndex << ","
                << agent[i]->memory.LCStartRouteIndex << ","
                << agent[i]->memory.LCDirection << ","
                << agent[i]->memory.checkSideVehicleForLC << ","
                << agent[i]->memory.LCCheckState << ","
                << agent[i]->memory.destinationNode << "\n";
        }
        else if( agent[i]->agentKind >= 100 ){

            out << agent[i]->state.x << ","
                << agent[i]->state.y << ","
                << agent[i]->state.z << ","
                << agent[i]->state.V << ","
                << agent[i]->state.yaw << ","
                << agent[i]->state.cosYaw << ","
                << agent[i]->state.sinYaw << ","
                << agent[i]->vehicle.vehicleModelID << "\n";

            out << agent[i]->memory.currentTargetPath << ","
                << agent[i]->memory.currentTargetPathIndexInList << "\n";
        }


        out << agent[i]->memory.accel << ","
            << agent[i]->memory.brake << ","
            << agent[i]->memory.steer << ","
            << agent[i]->memory.overrideBrakeByScenario << ","
            << agent[i]->memory.overrideAxControl << ","
            << agent[i]->memory.overrideSteerByScenario << ","
            << agent[i]->memory.overrideSteerControl << ","
            << agent[i]->memory.additionalShiftByScenarioEvent << ","
            << agent[i]->memory.additionalLateralShift << "\n";


        out << agent[i]->memory.controlMode << ","
            << agent[i]->memory.distanceToZeroSpeed << ","
            << agent[i]->memory.timeToZeroSpeed << ","
            << agent[i]->memory.requiredDistToStopFromTargetSpeed << ","
            << agent[i]->memory.minimumDistanceToStop << ","
            << agent[i]->memory.actualTargetSpeed << ","
            << agent[i]->memory.actualTargetHeadwayDistance << ","
            << agent[i]->memory.targetSpeed << ","
            << agent[i]->memory.targetHeadwayDistance << ","
            << agent[i]->memory.targetSpeedByScenario << ","
            << agent[i]->memory.actualTargetHeadwayDistanceByScenario << ","
            << agent[i]->memory.targetHeadwayDistanceByScenario << ","
            << agent[i]->memory.allowableHeadwayDistDeviation << ","
            << agent[i]->memory.targetHeadwayTimeByScenario << ","
            << agent[i]->memory.targetSpeedInsideIntersectionTurnByScenario << ","
            << agent[i]->memory.startDecel << ","
            << agent[i]->memory.activeBrakeInVelocityControl << "\n";


        out << agent[i]->memory.actualStopAtX << ","
            << agent[i]->memory.actualStopAtY << ","
            << agent[i]->memory.targetStopAtX << ","
            << agent[i]->memory.targetStopAtY << ","
            << agent[i]->memory.targetStopAtXByScenario << ","
            << agent[i]->memory.targetStopAtYByScenario << ","
            << agent[i]->memory.actualStopOnPathID << ","
            << agent[i]->memory.actualStopOnPathIndex << ","
            << agent[i]->memory.distToStopAtOnThePath << ","
            << agent[i]->memory.distanceToStopPoint << ","
            << agent[i]->memory.speedControlState << ","
            << agent[i]->memory.distanceAdjustLowSpeed << ","
            << agent[i]->memory.axSpeedControl << ","
            << agent[i]->memory.doHeadwayDistanceControl << ","
            << agent[i]->memory.axHeadwayControl << ","
            << agent[i]->memory.doStopControl << ","
            << agent[i]->memory.axStopControl << ","
            << agent[i]->memory.releaseStopCount << "\n";


        out << agent[i]->memory.doSteerControl << ","
            << agent[i]->memory.lateralDeviationFromTargetPathAtPreviewPoint << ","
            << agent[i]->memory.previewPointPath << ","
            << agent[i]->memory.lateralDeviationFromTargetPath << ","
            << agent[i]->memory.steeringControlGainMultiplier << ","
            << agent[i]->memory.lateralShiftTarget << ","
            << agent[i]->memory.avoidTarget << ","
            << agent[i]->memory.isChaningLane << ","
            << agent[i]->memory.speedProfileCount << ",";

        for(int j=0;j<agent[i]->memory.profileTime.size();++j){
            out << agent[i]->memory.profileTime[j];
            if( j < agent[i]->memory.profileTime.size() - 1 ){
                out << "/";
            }
        }
        out << ",";

        for(int j=0;j<agent[i]->memory.profileSpeed.size();++j){
            out << agent[i]->memory.profileSpeed[j];
            if( j < agent[i]->memory.profileSpeed.size() - 1 ){
                out << "/";
            }
        }
        out << "\n";


        out << agent[i]->memory.precedingVehicleID << ","
            << agent[i]->memory.precedingVehicleIDByScenario << ","
            << agent[i]->memory.distanceToPrecedingVehicle << ","
            << agent[i]->memory.speedPrecedingVehicle << ","
            << agent[i]->memory.axPrecedingVehicle << ","
            << agent[i]->memory.precedingObstacle << ","
            << agent[i]->memory.targetLateralShift << ","
            << agent[i]->memory.targetLateralShiftByScenario << ","
            << agent[i]->memory.distToNearOncomingCP << ","
            << agent[i]->memory.distToFatOncomingCP << ","
            << agent[i]->memory.shouldWaitOverCrossPoint << ","
            << agent[i]->memory.distToNearestCP << ","
            << agent[i]->memory.shouldStopAtSignalSL << ","
            << agent[i]->memory.distToYeildStopLine << ","
            << agent[i]->memory.shouldYeild << ","
            << agent[i]->memory.leftCrossIsClear << ","
            << agent[i]->memory.rightCrossIsClear << ","
            << agent[i]->memory.leftCrossCheckCount << ","
            << agent[i]->memory.rightCrossCheckCount << ","
            << agent[i]->memory.safetyConfimed << ","
            << agent[i]->memory.speedZeroCount << ","
            << agent[i]->memory.precedingVehicleIndex << "\n";

        for(int j=0;j<agent[i]->memory.perceptedObjects.size();++j){

            out << "PO" << "," << j << ",";

            out << agent[i]->memory.perceptedObjects[j]->objectID << ","
                << agent[i]->memory.perceptedObjects[j]->objectType << ","
                << agent[i]->memory.perceptedObjects[j]->vHalfLength << ","
                << agent[i]->memory.perceptedObjects[j]->vHalfWidth << ","
                << agent[i]->memory.perceptedObjects[j]->x << ","
                << agent[i]->memory.perceptedObjects[j]->y << ","
                << agent[i]->memory.perceptedObjects[j]->yaw << ","
                << agent[i]->memory.perceptedObjects[j]->V << ","
                << agent[i]->memory.perceptedObjects[j]->Ax << ","
                << agent[i]->memory.perceptedObjects[j]->objectPath << ","
                << agent[i]->memory.perceptedObjects[j]->objectTargetNode << ","
                << agent[i]->memory.perceptedObjects[j]->deviationFromObjectPath << ","
                << agent[i]->memory.perceptedObjects[j]->relPosEvaled << ","
                << agent[i]->memory.perceptedObjects[j]->nearestTargetPath << ","
                << agent[i]->memory.perceptedObjects[j]->deviationFromNearestTargetPath << ","
                << agent[i]->memory.perceptedObjects[j]->distanceToObject << ","
                << agent[i]->memory.perceptedObjects[j]->xOnTargetPath << ","
                << agent[i]->memory.perceptedObjects[j]->yOnTargetPath << ","
                << agent[i]->memory.perceptedObjects[j]->innerProductToNearestPathTangent << ","
                << agent[i]->memory.perceptedObjects[j]->innerProductToNearestPathNormal << ","
                << agent[i]->memory.perceptedObjects[j]->effectiveHalfWidth << ","
                << agent[i]->memory.perceptedObjects[j]->recognitionLabel << ","
                << agent[i]->memory.perceptedObjects[j]->objPathRecogLabelChecked << ","
                << agent[i]->memory.perceptedObjects[j]->myPathRecogLabelChecked << ","
                << agent[i]->memory.perceptedObjects[j]->winker << ","
                << agent[i]->memory.perceptedObjects[j]->hasCollisionPoint << ","
                << agent[i]->memory.perceptedObjects[j]->mergingAsCP << ","
                << agent[i]->memory.perceptedObjects[j]->xCP << ","
                << agent[i]->memory.perceptedObjects[j]->yCP << ","
                << agent[i]->memory.perceptedObjects[j]->myDistanceToCP << ","
                << agent[i]->memory.perceptedObjects[j]->myTimeToCP << ","
                << agent[i]->memory.perceptedObjects[j]->objectDistanceToCP << ","
                << agent[i]->memory.perceptedObjects[j]->objectTimeToCP << ","
                << agent[i]->memory.perceptedObjects[j]->CPinNode << ","
                << agent[i]->memory.perceptedObjects[j]->myCPPathIndex << ","
                << agent[i]->memory.perceptedObjects[j]->objCPPathIndex << ","
                << agent[i]->memory.perceptedObjects[j]->objPathCPChecked << ","
                << agent[i]->memory.perceptedObjects[j]->shouldEvalRisk << ","
                << agent[i]->memory.perceptedObjects[j]->inView << ","
                << agent[i]->memory.perceptedObjects[j]->noUpdateCount << ","
                << agent[i]->memory.perceptedObjects[j]->isValidData << "\n";
        }

        for(int j=0;j<agent[i]->memory.perceptedSignals.size();++j){

            out << "PS" << "," << j << ",";

            out << agent[i]->memory.perceptedSignals[j]->objectID << ","
                << agent[i]->memory.perceptedSignals[j]->objectType << ","
                << agent[i]->memory.perceptedSignals[j]->x << ","
                << agent[i]->memory.perceptedSignals[j]->y << ","
                << agent[i]->memory.perceptedSignals[j]->yaw << ","
                << agent[i]->memory.perceptedSignals[j]->relatedNode << ","
                << agent[i]->memory.perceptedSignals[j]->signalDisplay << ","
                << agent[i]->memory.perceptedSignals[j]->stopLineX << ","
                << agent[i]->memory.perceptedSignals[j]->stopLineY << ","
                << agent[i]->memory.perceptedSignals[j]->distToSL << ","
                << agent[i]->memory.perceptedSignals[j]->SLonPathID << ","
                << agent[i]->memory.perceptedSignals[j]->stopPointIndex << ","
                << agent[i]->memory.perceptedSignals[j]->inView << ","
                << agent[i]->memory.perceptedSignals[j]->noUpdateCount << ","
                << agent[i]->memory.perceptedSignals[j]->isValidData << "\n";
        }
    }


    for(int i=0;i<trafficSignal.size();++i){

        out << "TS," << i << ","
            << trafficSignal[i]->id << ","
            << trafficSignal[i]->currentDisplayIndex << ","
            << trafficSignal[i]->nextUpdateTimeFVal << ","
            << trafficSignal[i]->lastUpdateTimeFVal << ","
            << trafficSignal[i]->remainingTimeToNextDisplay << ","
            << trafficSignal[i]->elapsedTimeCurrentDisplay << ","
            << trafficSignal[i]->startOffset << ","
            << trafficSignal[i]->flushState << ","
            << trafficSignal[i]->flushTimeCheck << ",";

        for(int j=0;j<trafficSignal[i]->displayPattern.size();++j){
            out << trafficSignal[i]->displayPattern[j]->displayInfo << ":"
                << trafficSignal[i]->displayPattern[j]->duration;
            if( j < trafficSignal[i]->displayPattern.size() - 1 ){
                out << "/";
            }
        }
        out << "\n";
    }


    for(int i=0;i<road->odRoute.size();++i){

        out << "OD," << i << ","
            << road->odRoute[i]->originNode << ","
            << road->odRoute[i]->destinationNode << ","
            << road->odRoute[i]->meanArrivalTime << ","
            << road->odRoute[i]->NextAppearTime << "\n";

    }

    for(int i=0;i<road->pedestPaths.size();++i){

        out << "PP," << i << ","
               << road->pedestPaths[i]->meanArrivalTime << ","
               << road->pedestPaths[i]->NextAppearTime << "\n";

    }

    file.close();
}


void SystemThread::SetRestartData()
{
    if( restartFile == QString() ){
        return;
    }


    qDebug() << "[SystemThread::SetRestartData]";

    QFile file(restartFile);
    if( file.open(QIODevice::ReadOnly | QIODevice::Text) == false ){
        qDebug() << " Cannot open file: " << restartFile;
        return;
    }

    QTextStream in( &file );

    QString readLine;

    readLine = in.readLine();
    readLine = in.readLine();
    if( readLine.contains("Re:sim Restart Data") == false ){
        qDebug() << "Maybe the file is not Re:sim snapshot file.  read = " << readLine;
        file.close();
        return;
    }
    readLine = in.readLine();


    int cIdx = -1;
    int cPos = 0;

    int lineNo = 4;
    float simTimeInSecAtSnapshot = 0.0;

    while( in.atEnd() == false ){

        readLine = in.readLine();
        if( readLine.isEmpty() || readLine.startsWith("#") == true ){
            lineNo++;
            continue;
        }

        if( readLine.startsWith("Time-Stamp:") == true ){

            QStringList timeStrDiv = readLine.remove("Time-Stamp:").trimmed().split(":");

            int day  = QString(timeStrDiv[0]).remove("[d]").trimmed().toInt();
            int hour = QString(timeStrDiv[1]).remove("[h]").trimmed().toInt();
            int mint = QString(timeStrDiv[2]).remove("[m]").trimmed().toInt();
            float sec = QString(timeStrDiv[3]).remove("[s]").trimmed().toFloat();

            if( DSMode == false ){
                simManage->SetSimulationTime(day,hour,mint,sec);
            }
            else{
                simTimeInSecAtSnapshot = day * 86400.0 + hour * 3600.0 + mint * 60.0 + sec;
            }

            lineNo++;
            continue;
        }
        else if( readLine.startsWith("A,") == true ){

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 14 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }



            int idx                     =  QString(valDiv[1]).trimmed().toInt();
            int ID                      =  QString(valDiv[2]).trimmed().toInt();

//            qDebug() << "A; ID=" << ID;

            int agentKind               =  QString(valDiv[3]).trimmed().toInt();
            bool isScenarioObject       = (QString(valDiv[4]).trimmed().toInt() == 1 ? true : false);
            bool isBehaviorEmbeded      = (QString(valDiv[5]).trimmed().toInt() == 1 ? true : false);
            bool isSInterfaceObject     = (QString(valDiv[6]).trimmed().toInt() == 1 ? true : false);
            bool notAllowedAppear       = (QString(valDiv[7]).trimmed().toInt() == 1 ? true : false);
            float calInterval           =  QString(valDiv[8]).trimmed().toFloat();
            int cognitionCountMax       =  QString(valDiv[9]).trimmed().toInt();
            int cognitionCount          =  QString(valDiv[10]).trimmed().toInt();
            int decisionMakingCountMax  =  QString(valDiv[11]).trimmed().toInt();
            int decisionMakingCount     =  QString(valDiv[12]).trimmed().toInt();
            int controlCountMax         =  QString(valDiv[13]).trimmed().toInt();
            int controlCount            =  QString(valDiv[14]).trimmed().toInt();


            cIdx = idx;

            agent[cIdx]->InitializeMemory();

            agent[cIdx]->ID                     = ID;
            agent[cIdx]->agentKind              = agentKind;
            agent[cIdx]->isScenarioObject       = isScenarioObject;
            agent[cIdx]->isBehaviorEmbeded      = isBehaviorEmbeded;
            agent[cIdx]->isSInterfaceObject     = isSInterfaceObject;
            agent[cIdx]->notAllowedAppear       = notAllowedAppear;

            if( DSMode == false ){
                agent[cIdx]->calInterval = calInterval;
            }
            else{
                agent[cIdx]->calInterval = simManage->GetCalculationInterval();

                float dt = simManage->GetCalculationInterval();

                if( agent[cIdx]->agentKind < 100 ){

                    if( agent[cIdx]->vehicle.steerFiltered4CG == NULL ){
                        agent[cIdx]->vehicle.steerFiltered4CG = new LowPassFilter(1, dt, 6.0 , 0.0);
                    }
                    else{
                        agent[cIdx]->vehicle.steerFiltered4CG->SetParam(1, dt, 6.0 , 0.0);
                        agent[cIdx]->vehicle.steerFiltered4CG->Reset();
                    }

                    if( agent[cIdx]->vehicle.axFiltered4CG == NULL ){
                        agent[cIdx]->vehicle.axFiltered4CG = new LowPassFilter(1, dt, 6.0 , 0.0);
                    }
                    else{
                        agent[cIdx]->vehicle.axFiltered4CG->SetParam(1, dt, 6.0 , 0.0);
                        agent[cIdx]->vehicle.axFiltered4CG->Reset();
                    }

                    if( agent[cIdx]->vehicle.ayFiltered4CG == NULL ){
                        agent[cIdx]->vehicle.ayFiltered4CG = new LowPassFilter(1, dt, 6.0 , 0.0);
                    }
                    else{
                        agent[cIdx]->vehicle.ayFiltered4CG->SetParam(1, dt, 6.0 , 0.0);
                        agent[cIdx]->vehicle.ayFiltered4CG->Reset();
                    }

                    if( agent[cIdx]->vehicle.suspentionFL4CG == NULL ){
                        agent[cIdx]->vehicle.suspentionFL4CG = new LowPassFilter(2, dt, 10.0 , 0.6);
                    }
                    else{
                        agent[cIdx]->vehicle.suspentionFL4CG->SetParam(2, dt, 10.0 , 0.6);
                        agent[cIdx]->vehicle.suspentionFL4CG->Reset();
                    }

                    if( agent[cIdx]->vehicle.suspentionFR4CG == NULL ){
                        agent[cIdx]->vehicle.suspentionFR4CG = new LowPassFilter(2, dt, 10.0 , 0.6);
                    }
                    else{
                        agent[cIdx]->vehicle.suspentionFR4CG->SetParam(2, dt, 10.0 , 0.6);
                        agent[cIdx]->vehicle.suspentionFR4CG->Reset();
                    }

                    if( agent[cIdx]->vehicle.suspentionRL4CG == NULL ){
                        agent[cIdx]->vehicle.suspentionRL4CG = new LowPassFilter(2, dt, 10.0 , 0.6);
                    }
                    else{
                        agent[cIdx]->vehicle.suspentionRL4CG->SetParam(2, dt, 10.0 , 0.6);
                        agent[cIdx]->vehicle.suspentionRL4CG->Reset();
                    }

                    if( agent[cIdx]->vehicle.suspentionRR4CG == NULL ){
                        agent[cIdx]->vehicle.suspentionRR4CG = new LowPassFilter(2, dt, 10.0 , 0.6);
                    }
                    else{
                        agent[cIdx]->vehicle.suspentionRR4CG->SetParam(2, dt, 10.0 , 0.6);
                        agent[cIdx]->vehicle.suspentionRR4CG->Reset();
                    }

                    if( agent[cIdx]->vehicle.tireFL4CG == NULL ){
                        agent[cIdx]->vehicle.tireFL4CG = new LowPassFilter(1, dt, 5.0 , 0.0);
                    }
                    else{
                        agent[cIdx]->vehicle.tireFL4CG->SetParam(1, dt, 5.0 , 0.0);
                        agent[cIdx]->vehicle.tireFL4CG->Reset();
                    }

                    if( agent[cIdx]->vehicle.tireFR4CG == NULL ){
                        agent[cIdx]->vehicle.tireFR4CG = new LowPassFilter(1, dt, 5.0 , 0.0);
                    }
                    else{
                        agent[cIdx]->vehicle.tireFR4CG->SetParam(1, dt, 5.0 , 0.0);
                        agent[cIdx]->vehicle.tireFR4CG->Reset();
                    }

                    if( agent[cIdx]->vehicle.tireRL4CG == NULL ){
                        agent[cIdx]->vehicle.tireRL4CG = new LowPassFilter(1, dt, 5.0 , 0.0);
                    }
                    else{
                        agent[cIdx]->vehicle.tireRL4CG->SetParam(1, dt, 5.0 , 0.0);
                        agent[cIdx]->vehicle.tireRL4CG->Reset();
                    }

                    if( agent[cIdx]->vehicle.tireRR4CG == NULL ){
                        agent[cIdx]->vehicle.tireRR4CG = new LowPassFilter(1, dt, 5.0 , 0.0);
                    }
                    else{
                        agent[cIdx]->vehicle.tireRR4CG->SetParam(1, dt, 5.0 , 0.0);
                        agent[cIdx]->vehicle.tireRR4CG->Reset();
                    }

                    if( agent[cIdx]->vehicle.yawFiltered4CG == NULL ){
                        agent[cIdx]->vehicle.yawFiltered4CG = new LowPassFilter(1, dt, 3.0 , 0.0);
                    }
                    agent[cIdx]->vehicle.yawFiltered4CG->SetParam(1,dt, 3.0 , 0.0);
                }
                else if( agent[cIdx]->agentKind >= 100 ){

                    if( agent[cIdx]->vehicle.yawFiltered4CG == NULL ){
                        agent[cIdx]->vehicle.yawFiltered4CG = new LowPassFilter(1, dt, 10.0 , 0.0);
                    }
                    agent[cIdx]->vehicle.yawFiltered4CG->SetParam(1,dt, 10.0 , 0.0);
                    agent[cIdx]->vehicle.yawFiltered4CG->SetInitialValue( agent[cIdx]->state.yaw );

                    if( agent[cIdx]->vehicle.axFiltered4CG == NULL ){
                        agent[cIdx]->vehicle.axFiltered4CG = new LowPassFilter(1, dt, 6.0 , 0.0);
                    }

                    agent[cIdx]->vehicle.axFiltered4CG->SetParam(1, dt, 30.0 , 0.0);
                    agent[cIdx]->vehicle.axFiltered4CG->SetInitialValue( cos( agent[cIdx]->state.yaw ) );

                    if( agent[cIdx]->vehicle.ayFiltered4CG == NULL ){
                        agent[cIdx]->vehicle.ayFiltered4CG = new LowPassFilter(1, dt, 6.0 , 0.0);
                    }

                    agent[cIdx]->vehicle.ayFiltered4CG->SetParam(1, dt, 30.0 , 0.0);
                    agent[cIdx]->vehicle.ayFiltered4CG->SetInitialValue( sin( agent[cIdx]->state.yaw ) );
                }


            }

            agent[cIdx]->cognitionCountMax      = cognitionCountMax;
            agent[cIdx]->cognitionCount         = cognitionCount;
            agent[cIdx]->decisionMakingCountMax = decisionMakingCountMax;
            agent[cIdx]->decisionMakingCount    = decisionMakingCount;
            agent[cIdx]->controlCountMax        = controlCountMax;
            agent[cIdx]->controlCount           = controlCount;

            agent[cIdx]->vehicle.SetVehicleID( ID );

            agent[cIdx]->agentStatus = 1;

            lineNo++;
            cPos = 1;
            continue;
        }
        else if( cIdx >= 0 && cPos == 1 ){

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 1 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }

//            qDebug() << " cPos=" << cPos;

            int age    = QString(valDiv[0]).trimmed().toInt();
            int gender = QString(valDiv[1]).trimmed().toInt();

            agent[cIdx]->attri.age    = age;
            agent[cIdx]->attri.gender = gender;

            lineNo++;
            cPos = 2;
            continue;
        }
        else if( cIdx >= 0 && cPos == 2 ){

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 16 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }

//            qDebug() << " cPos=" << cPos;

            agent[cIdx]->param.startRelay =  QString(valDiv[0]).trimmed().toFloat();
            agent[cIdx]->param.headwayTime =  QString(valDiv[1]).trimmed().toFloat();
            agent[cIdx]->param.latAccelAtTurn =  QString(valDiv[2]).trimmed().toFloat();
            agent[cIdx]->param.maxDeceleration =  QString(valDiv[3]).trimmed().toFloat();
            agent[cIdx]->param.visibleDistance =  QString(valDiv[4]).trimmed().toFloat();
            agent[cIdx]->param.accelControlGain =  QString(valDiv[5]).trimmed().toFloat();
            agent[cIdx]->param.safetyConfirmTime =  QString(valDiv[6]).trimmed().toFloat();
            agent[cIdx]->param.headwayControlGain =  QString(valDiv[7]).trimmed().toFloat();
            agent[cIdx]->param.steeringControlGain =  QString(valDiv[8]).trimmed().toFloat();
            agent[cIdx]->param.accelOffDeceleration =  QString(valDiv[9]).trimmed().toFloat();
            agent[cIdx]->param.deadZoneSpeedControl =  QString(valDiv[10]).trimmed().toFloat();
            agent[cIdx]->param.crossTimeSafetyMargin =  QString(valDiv[11]).trimmed().toFloat();
            agent[cIdx]->param.minimumDistanceToStopLine =  QString(valDiv[12]).trimmed().toFloat();
            agent[cIdx]->param.crossWaitPositionSafeyMargin =  QString(valDiv[13]).trimmed().toFloat();
            agent[cIdx]->param.minimumHeadwayDistanceAtStop =  QString(valDiv[14]).trimmed().toFloat();
            agent[cIdx]->param.pedestWaitPositionSafetyMargin =  QString(valDiv[15]).trimmed().toFloat();
            agent[cIdx]->param.minimumPerceptibleDecelerationOfPreceding =  QString(valDiv[16]).trimmed().toFloat();

            if( valDiv.size() >= 18 ){
                agent[cIdx]->param.speedVariationFactor = QString(valDiv[17]).trimmed().toFloat();
            }
            else{
                agent[cIdx]->param.speedVariationFactor = simManage->GetNormalDist(0.0,0.3);
                if( agent[cIdx]->param.speedVariationFactor > 1.5 ){
                    agent[cIdx]->param.speedVariationFactor = 1.5;
                }
                else if( agent[cIdx]->param.speedVariationFactor < -0.8 ){
                    agent[cIdx]->param.speedVariationFactor = -0.8;
                }

                agent[cIdx]->param.latAccelAtTurn = ( 0.20 + agent[cIdx]->param.speedVariationFactor * 0.07) * 9.81;
            }


            lineNo++;
            cPos = 3;
            continue;
        }
        else if( cIdx >= 0 && cPos == 3 ){

//            qDebug() << " cPos=" << cPos;

            if( agent[cIdx]->agentKind < 100 ){

                QStringList valDiv = readLine.trimmed().split(",");
                if( valDiv.size() <= 14 ){
                    qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                    return;
                }

                agent[cIdx]->vehicle.state.X        =  QString(valDiv[0]).trimmed().toFloat();
                agent[cIdx]->vehicle.state.Y        =  QString(valDiv[1]).trimmed().toFloat();
                agent[cIdx]->vehicle.state.Z        =  QString(valDiv[2]).trimmed().toFloat();
                agent[cIdx]->vehicle.state.vx       =  QString(valDiv[3]).trimmed().toFloat();
                agent[cIdx]->vehicle.state.yawAngle =  QString(valDiv[4]).trimmed().toFloat();
                agent[cIdx]->vehicle.state.roll     =  QString(valDiv[5]).trimmed().toFloat();
                agent[cIdx]->vehicle.state.pitch    =  QString(valDiv[6]).trimmed().toFloat();
                agent[cIdx]->vehicle.state.yawRate  =  QString(valDiv[7]).trimmed().toFloat();
                agent[cIdx]->vehicle.state.ax       =  QString(valDiv[8]).trimmed().toFloat();
                agent[cIdx]->state.accel            =  QString(valDiv[9]).trimmed().toFloat();
                agent[cIdx]->state.brake            =  QString(valDiv[10]).trimmed().toFloat();
                agent[cIdx]->state.steer            =  QString(valDiv[11]).trimmed().toFloat();
                agent[cIdx]->vehicle.winker_state   =  QString(valDiv[12]).trimmed().toInt();
                agent[cIdx]->vehicle.winker_count   =  QString(valDiv[13]).trimmed().toInt();
                agent[cIdx]->vehicle.vehicleModelID =  QString(valDiv[14]).trimmed().toInt();

                int j = agent[cIdx]->vehicle.vehicleModelID;

                float L = road->vehicleKind[j]->length;
                float W = road->vehicleKind[j]->width;
                float H = road->vehicleKind[j]->height;

                agent[cIdx]->objTypeForUE4 = road->vehicleKind[j]->type;
                agent[cIdx]->objNoForUE4   = road->vehicleKind[j]->No;
                if( road->vehicleKind[j]->UE4ModelID > 0){
                    agent[cIdx]->objIDForUE4 = road->vehicleKind[j]->UE4ModelID;
                }
                else{
                    agent[cIdx]->objIDForUE4 = -1;
                }

                agent[cIdx]->vehicle.SetVehicleParam( L, W, H, L * 0.8, L * 0.1, 0.5 );
                agent[cIdx]->vHalfLength = L * 0.5;
                agent[cIdx]->vHalfWidth  = W  * 0.5;

                agent[cIdx]->state.V = agent[cIdx]->vehicle.state.vx;
                agent[cIdx]->state.x = agent[cIdx]->vehicle.state.X;
                agent[cIdx]->state.y = agent[cIdx]->vehicle.state.Y;
                agent[cIdx]->state.z = agent[cIdx]->vehicle.state.Z;
                agent[cIdx]->state.yaw = agent[cIdx]->vehicle.state.yawAngle;
                agent[cIdx]->state.cosYaw = cos( agent[cIdx]->vehicle.state.yawAngle );
                agent[cIdx]->state.sinYaw = sin( agent[cIdx]->vehicle.state.yawAngle );

                if( DSMode == true ){
                    agent[cIdx]->vehicle.yawFiltered4CG->SetInitialValue( agent[cIdx]->state.yaw );
                }
            }
            else if( agent[cIdx]->agentKind >= 100 ){

                QStringList valDiv = readLine.trimmed().split(",");
                if( valDiv.size() <= 7 ){
                    qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                    return;
                }

                agent[cIdx]->state.x = QString(valDiv[0]).trimmed().toFloat();
                agent[cIdx]->state.y = QString(valDiv[1]).trimmed().toFloat();
                agent[cIdx]->state.z = QString(valDiv[2]).trimmed().toFloat();
                agent[cIdx]->state.V = QString(valDiv[3]).trimmed().toFloat();
                agent[cIdx]->state.yaw = QString(valDiv[4]).trimmed().toFloat();
                agent[cIdx]->state.cosYaw = QString(valDiv[5]).trimmed().toFloat();
                agent[cIdx]->state.sinYaw = QString(valDiv[6]).trimmed().toFloat();
                agent[cIdx]->vehicle.vehicleModelID =  QString(valDiv[7]).trimmed().toInt();

                agent[cIdx]->state.z_path = agent[cIdx]->state.z;
                agent[cIdx]->state.pitch  = 0.0;
                agent[cIdx]->state.roll   = 0.0;

                agent[cIdx]->vehicle.state.vx = agent[cIdx]->state.V;
                agent[cIdx]->vehicle.state.X = agent[cIdx]->state.x;
                agent[cIdx]->vehicle.state.Y = agent[cIdx]->state.y;
                agent[cIdx]->vehicle.state.Z = agent[cIdx]->state.z;
                agent[cIdx]->vehicle.state.yawAngle = agent[cIdx]->state.yaw;

                int j = agent[cIdx]->vehicle.vehicleModelID;
                float L = road->pedestrianKind[j]->length;
                float W = road->pedestrianKind[j]->width;
                float H = road->pedestrianKind[j]->height;

                agent[cIdx]->objTypeForUE4 = road->pedestrianKind[j]->type;
                agent[cIdx]->objNoForUE4   = road->pedestrianKind[j]->No;
                if( road->pedestrianKind[j]->UE4ModelID > 0){
                    agent[cIdx]->objIDForUE4 = road->pedestrianKind[j]->UE4ModelID;
                }
                else{
                    agent[cIdx]->objIDForUE4 = -1;
                }

                agent[cIdx]->vehicle.SetVehicleParam( L, W, H, L * 0.8, L * 0.1, 0.5 );
                agent[cIdx]->vHalfLength = L * 0.5;
                agent[cIdx]->vHalfWidth  = W  * 0.5;


                if( DSMode == true ){
                    agent[cIdx]->vehicle.yawFiltered4CG->SetInitialValue( agent[cIdx]->state.yaw );
                    agent[cIdx]->vehicle.axFiltered4CG->SetInitialValue( cos( agent[cIdx]->state.yaw ) );
                    agent[cIdx]->vehicle.ayFiltered4CG->SetInitialValue( sin( agent[cIdx]->state.yaw ) );
                }
            }

            lineNo++;
            cPos = 4;
            continue;
        }
        else if( cIdx >= 0 && cPos == 4 ){

//            qDebug() << " cPos=" << cPos;


            if( agent[cIdx]->agentKind < 100 ){

                QStringList valDiv = readLine.trimmed().split(",");
                if( valDiv.size() <= 10 ){
                    qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                    return;
                }

                agent[cIdx]->memory.routeType                        =  QString(valDiv[0]).trimmed().toInt();
                agent[cIdx]->memory.routeIndex                       =  QString(valDiv[1]).trimmed().toInt();
                agent[cIdx]->memory.routeLaneIndex                   =  QString(valDiv[2]).trimmed().toInt();
                agent[cIdx]->memory.currentTargetPath                =  QString(valDiv[3]).trimmed().toInt();
                agent[cIdx]->memory.currentTargetPathIndexInList     =  QString(valDiv[4]).trimmed().toInt();
                agent[cIdx]->memory.distanceFromStartWPInCurrentPath =  QString(valDiv[5]).trimmed().toFloat();
                agent[cIdx]->memory.scenarioPathSelectID             =  QString(valDiv[6]).trimmed().toInt();
                agent[cIdx]->memory.distanceToTurnNodeWPIn           =  QString(valDiv[7]).trimmed().toFloat();
                agent[cIdx]->memory.distanceToNodeWPIn               =  QString(valDiv[8]).trimmed().toFloat();
                agent[cIdx]->memory.distanceToTurnNodeWPOut          =  QString(valDiv[9]).trimmed().toFloat();
                agent[cIdx]->memory.distanceToNodeWPOut              =  QString(valDiv[10]).trimmed().toFloat();
                if( valDiv.size() >= 16 ){
                    agent[cIdx]->memory.LCSupportRouteLaneIndex      =  QString(valDiv[11]).trimmed().toInt();
                    agent[cIdx]->memory.LCStartRouteIndex            =  QString(valDiv[12]).trimmed().toInt();
                    agent[cIdx]->memory.LCDirection                  =  QString(valDiv[13]).trimmed().toInt();
                    agent[cIdx]->memory.checkSideVehicleForLC        = (QString(valDiv[14]).trimmed().toInt() == 1 ? true : false);
                    agent[cIdx]->memory.LCCheckState                 =  QString(valDiv[15]).trimmed().toInt();
                }
                else{
                    agent[cIdx]->memory.LCSupportRouteLaneIndex      =  -1;
                    agent[cIdx]->memory.LCStartRouteIndex            =  -1;
                    agent[cIdx]->memory.LCDirection                  =  0;
                    agent[cIdx]->memory.checkSideVehicleForLC        =  false;
                    agent[cIdx]->memory.LCCheckState                 =  0;
                }
                if( valDiv.size() >= 17 ){
                    agent[cIdx]->memory.destinationNode = QString(valDiv[16]).trimmed().toInt();
                }
                else{
                    agent[cIdx]->memory.destinationNode = -1;
                }

                int i = agent[cIdx]->memory.routeIndex;
                int selIdx = agent[cIdx]->memory.routeLaneIndex;

                if( i >= 0 && i < road->odRoute.size() &&
                        road->odRoute[i]->laneListsToDestination.size() > 0 && selIdx < road->odRoute[i]->laneListsToDestination.size() ){  // old ver, to be deleted

                    agent[cIdx]->memory.targetPathList = road->odRoute[i]->laneListsToDestination[selIdx];

                    agent[cIdx]->memory.currentTargetPathIndexInList = -1;
                    for(int n = agent[cIdx]->memory.targetPathList.size()-1 ; n>=0 ; n--){
                        if( agent[cIdx]->memory.currentTargetPathIndexInList < 0 &&
                                agent[cIdx]->memory.targetPathList[n] != agent[cIdx]->memory.currentTargetPath ){
                            continue;
                        }
                        agent[cIdx]->memory.currentTargetPathIndexInList = n;
                        break;
                    }

                    agent[cIdx]->memory.targetPathLength.clear();
                    for(int nn=0;nn<agent[cIdx]->memory.targetPathList.size();++nn){
                        float len = road->GetPathLength( agent[cIdx]->memory.targetPathList[nn] );
                        agent[cIdx]->memory.targetPathLength.append( len );
                    }

                    agent[cIdx]->memory.laneMerge.clear();

                    for(int k=0;k<road->odRoute[i]->mergeLanesInfo[selIdx].size();++k){

                        QPoint pairData;
                        pairData.setX( road->odRoute[i]->mergeLanesInfo[selIdx][k].x() );
                        pairData.setY( road->odRoute[i]->mergeLanesInfo[selIdx][k].y() );

                        agent[cIdx]->memory.laneMerge.append( pairData );
                    }
                }
                else{  // new ver

                    bool isSet = false;

                    // OD data can be difference for current and snapshot
                    if( i >= 0 && i < road->odRoute.size() &&
                            road->odRoute[i]->destinationNode == agent[cIdx]->memory.destinationNode ){

                        int j = agent[cIdx]->memory.LCSupportRouteLaneIndex;
                        if( j >= 0 && j < road->odRoute[i]->LCSupportLaneLists.size() &&
                                selIdx >= 0 && selIdx < road->odRoute[i]->LCSupportLaneLists[j]->laneList.size() &&
                                road->odRoute[i]->LCSupportLaneLists[j]->laneList[selIdx].indexOf( agent[cIdx]->memory.currentTargetPath ) >= 0 ){

                            agent[cIdx]->memory.targetPathList = road->odRoute[i]->LCSupportLaneLists[j]->laneList[selIdx];

                            agent[cIdx]->memory.currentTargetPathIndexInList = -1;
                            for(int n = agent[cIdx]->memory.targetPathList.size()-1 ; n>=0 ; n--){
                                if( agent[cIdx]->memory.currentTargetPathIndexInList < 0 &&
                                        agent[cIdx]->memory.targetPathList[n] != agent[cIdx]->memory.currentTargetPath ){
                                    continue;
                                }
                                agent[cIdx]->memory.currentTargetPathIndexInList = n;
                                break;
                            }

                            agent[cIdx]->memory.targetPathLength.clear();
                            for(int nn=0;nn<agent[cIdx]->memory.targetPathList.size();++nn){
                                float len = road->GetPathLength( agent[cIdx]->memory.targetPathList[nn] );
                                agent[cIdx]->memory.targetPathLength.append( len );
                            }

                            agent[cIdx]->memory.laneMerge.clear();

                            int MLIidx = 0;
                            for(int n=0;n<j;++n){
                                MLIidx += road->odRoute[i]->LCSupportLaneLists[n]->laneList.size();
                            }
                            MLIidx += selIdx;

                            for(int k=0;k<road->odRoute[i]->mergeLanesInfo[MLIidx].size();++k){

                                QPoint pairData;
                                pairData.setX( road->odRoute[i]->mergeLanesInfo[MLIidx][k].x() );
                                pairData.setY( road->odRoute[i]->mergeLanesInfo[MLIidx][k].y() );

                                agent[cIdx]->memory.laneMerge.append( pairData );
                            }

                            agent[cIdx]->SetTargetSpeedIndividual( road->paths[ road->pathId2Index.indexOf(agent[cIdx]->memory.currentTargetPath) ]->speed85pt );
                            agent[cIdx]->SetTargetNodeListByTargetPaths( road );

                            isSet = true;
                        }
                    }


                    if( isSet == false ){

                        // Try ro find the route contains current path
                        QStringList validRoutes;
                        QList<int> rIdx;
                        for(int k=0;k<road->odRoute.size();++k){
                            for(int l=0;l<road->odRoute[k]->LCSupportLaneLists.size();++l){
                                for(int m=0;m<road->odRoute[k]->LCSupportLaneLists[l]->laneList.size();++m){
                                    if( road->odRoute[k]->LCSupportLaneLists[l]->laneList[m].indexOf( agent[cIdx]->memory.currentTargetPath ) >= 0 ){
                                        validRoutes.append( QString("%1,%2,%3").arg(k).arg(l).arg(m) );
                                        rIdx.append(k);
                                    }
                                }
                            }
                        }

                        if( validRoutes.size() == 0 ){
                            // No route found, should disappear
                            agent[cIdx]->agentStatus = 2;

                            lineNo++;
                            cPos = 5;
                            continue;
                        }

                        // Try ro find the route for original destination
                        for(int k=rIdx.size()-1;k>=0;--k){
                            if( road->odRoute[rIdx[k]]->destinationNode != agent[cIdx]->memory.destinationNode ){
                                rIdx.removeAt(k);
                            }
                        }

                        if( rIdx.size() > 0 ){

                            int n = rIdx.size();
                            int k = (int)(simManage->rndGen.GenUniform() * n);
                            if( k >= n ){
                                k = n - 1;
                            }

                            int m = 0;
                            for(int l=0;l<validRoutes.size();++l){
                                if( QString(validRoutes[l]).startsWith(QString("%1,").arg(rIdx[k])) == true ){
                                    if( m == k ){

                                        agent[cIdx]->memory.routeIndex = rIdx[k];

                                        QStringList div = QString(validRoutes[l]).split(",");

                                        agent[cIdx]->memory.LCSupportRouteLaneIndex = QString( div[1] ).trimmed().toInt();
                                        agent[cIdx]->memory.routeLaneIndex = QString( div[2] ).trimmed().toInt();


                                        int kk = agent[cIdx]->memory.routeIndex;
                                        int ll = agent[cIdx]->memory.LCSupportRouteLaneIndex;
                                        int mm = agent[cIdx]->memory.routeLaneIndex;

                                        if( agent[cIdx]->memory.LCSupportRouteLaneIndex == 0 ){
                                            agent[cIdx]->memory.LCStartRouteIndex = -1;
                                        }
                                        else{
                                            agent[cIdx]->memory.LCStartRouteIndex = road->odRoute[kk]->LCSupportLaneLists[ll]->gIndexInNodeList;
                                        }

                                        agent[cIdx]->memory.targetPathList = road->odRoute[kk]->LCSupportLaneLists[ll]->laneList[mm];

                                        agent[cIdx]->memory.currentTargetPathIndexInList = -1;
                                        for(int nn = agent[cIdx]->memory.targetPathList.size()-1 ; nn>=0 ; nn--){
                                            if( agent[cIdx]->memory.currentTargetPathIndexInList < 0 &&
                                                    agent[cIdx]->memory.targetPathList[nn] != agent[cIdx]->memory.currentTargetPath ){
                                                continue;
                                            }
                                            agent[cIdx]->memory.currentTargetPathIndexInList = nn;
                                            break;
                                        }

                                        agent[cIdx]->memory.targetPathLength.clear();
                                        for(int nn=0;nn<agent[cIdx]->memory.targetPathList.size();++nn){
                                            float len = road->GetPathLength( agent[cIdx]->memory.targetPathList[nn] );
                                            agent[cIdx]->memory.targetPathLength.append( len );
                                        }


                                        agent[cIdx]->memory.laneMerge.clear();

                                        int MLIidx = 0;
                                        for(int nn=0;nn<ll;++nn){
                                            MLIidx += road->odRoute[kk]->LCSupportLaneLists[nn]->laneList.size();
                                        }
                                        MLIidx += mm;

                                        for(int nn=0;nn<road->odRoute[kk]->mergeLanesInfo[MLIidx].size();++nn){

                                            QPoint pairData;
                                            pairData.setX( road->odRoute[kk]->mergeLanesInfo[MLIidx][nn].x() );
                                            pairData.setY( road->odRoute[kk]->mergeLanesInfo[MLIidx][nn].y() );

                                            agent[cIdx]->memory.laneMerge.append( pairData );
                                        }

                                        agent[cIdx]->SetTargetSpeedIndividual( road->paths[ road->pathId2Index.indexOf(agent[cIdx]->memory.currentTargetPath) ]->speed85pt );
                                        agent[cIdx]->SetTargetNodeListByTargetPaths( road );

                                        isSet = true;

                                        break;
                                    }
                                    else{
                                        m++;
                                    }
                                }
                            }

                        }
                        else{

                            int n = validRoutes.size();
                            if( n == 0 ){
                                // No route found, should disappear
                                agent[cIdx]->agentStatus = 2;

                                lineNo++;
                                cPos = 5;
                                continue;
                            }

                            int k = (int)(simManage->rndGen.GenUniform() * n);
                            if( k >= n ){
                                k = n - 1;
                            }

                            QStringList div = QString(validRoutes[k]).split(",");

                            int kk = QString( div[0] ).trimmed().toInt();
                            int ll = QString( div[1] ).trimmed().toInt();
                            int mm = QString( div[2] ).trimmed().toInt();

                            agent[cIdx]->memory.destinationNode = road->odRoute[kk]->destinationNode;

                            agent[cIdx]->memory.routeIndex = kk;
                            agent[cIdx]->memory.LCSupportRouteLaneIndex = ll;
                            agent[cIdx]->memory.routeLaneIndex = mm;

                            if( ll == 0 ){
                                agent[cIdx]->memory.LCStartRouteIndex = -1;
                            }
                            else{
                                agent[cIdx]->memory.LCStartRouteIndex = road->odRoute[kk]->LCSupportLaneLists[ll]->gIndexInNodeList;
                            }

                            agent[cIdx]->memory.targetPathList = road->odRoute[kk]->LCSupportLaneLists[ll]->laneList[mm];

                            agent[cIdx]->memory.currentTargetPathIndexInList = -1;
                            for(int nn = agent[cIdx]->memory.targetPathList.size()-1 ; nn>=0 ; nn--){
                                if( agent[cIdx]->memory.currentTargetPathIndexInList < 0 &&
                                        agent[cIdx]->memory.targetPathList[nn] != agent[cIdx]->memory.currentTargetPath ){
                                    continue;
                                }
                                agent[cIdx]->memory.currentTargetPathIndexInList = nn;
                                break;
                            }

                            agent[cIdx]->memory.targetPathLength.clear();
                            for(int nn=0;nn<agent[cIdx]->memory.targetPathList.size();++nn){
                                float len = road->GetPathLength( agent[cIdx]->memory.targetPathList[nn] );
                                agent[cIdx]->memory.targetPathLength.append( len );
                            }


                            agent[cIdx]->memory.laneMerge.clear();

                            int MLIidx = 0;
                            for(int nn=0;nn<ll;++nn){
                                MLIidx += road->odRoute[kk]->LCSupportLaneLists[nn]->laneList.size();
                            }
                            MLIidx += mm;

                            for(int nn=0;nn<road->odRoute[kk]->mergeLanesInfo[MLIidx].size();++nn){

                                QPoint pairData;
                                pairData.setX( road->odRoute[kk]->mergeLanesInfo[MLIidx][nn].x() );
                                pairData.setY( road->odRoute[kk]->mergeLanesInfo[MLIidx][nn].y() );

                                agent[cIdx]->memory.laneMerge.append( pairData );
                            }

                            agent[cIdx]->SetTargetSpeedIndividual( road->paths[ road->pathId2Index.indexOf(agent[cIdx]->memory.currentTargetPath) ]->speed85pt );
                            agent[cIdx]->SetTargetNodeListByTargetPaths( road );

                            isSet = true;
                        }
                    }

                    if( isSet == false ){
                        agent[cIdx]->agentStatus = 2;

                        lineNo++;
                        cPos = 5;
                        continue;
                    }
                }

            }
            else if( agent[cIdx]->agentKind >= 100 ){

                QStringList valDiv = readLine.trimmed().split(",");
                if( valDiv.size() <= 1 ){
                    qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                    return;
                }

                agent[cIdx]->memory.currentTargetPath                =  QString(valDiv[0]).trimmed().toInt();
                agent[cIdx]->memory.currentTargetPathIndexInList     =  QString(valDiv[1]).trimmed().toInt();

                agent[cIdx]->memory.targetPathList.clear();
                agent[cIdx]->memory.targetPathList.append( agent[cIdx]->memory.currentTargetPath );

            }


            lineNo++;
            cPos = 5;
            continue;
        }
        else if( cIdx >= 0 && cPos == 5 ){

//            qDebug() << " cPos=" << cPos;

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 8 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }


            agent[cIdx]->memory.accel  =  QString(valDiv[0]).trimmed().toFloat();
            agent[cIdx]->memory.brake  =  QString(valDiv[1]).trimmed().toFloat();
            agent[cIdx]->memory.steer  =  QString(valDiv[2]).trimmed().toFloat();

            agent[cIdx]->memory.overrideBrakeByScenario        = (QString(valDiv[3]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.overrideAxControl              =  QString(valDiv[4]).trimmed().toFloat();
            agent[cIdx]->memory.overrideSteerByScenario        = (QString(valDiv[5]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.overrideSteerControl           =  QString(valDiv[6]).trimmed().toFloat();
            agent[cIdx]->memory.additionalShiftByScenarioEvent = (QString(valDiv[7]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.additionalLateralShift         =  QString(valDiv[8]).trimmed().toFloat();


            lineNo++;
            cPos = 6;
            continue;
        }
        else if( cIdx >= 0 && cPos == 6 ){

//            qDebug() << " cPos=" << cPos;

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 16 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }

            agent[cIdx]->memory.controlMode                                 =  QString(valDiv[0]).trimmed().toInt();
            agent[cIdx]->memory.distanceToZeroSpeed                         =  QString(valDiv[1]).trimmed().toFloat();
            agent[cIdx]->memory.timeToZeroSpeed                             =  QString(valDiv[2]).trimmed().toFloat();
            agent[cIdx]->memory.requiredDistToStopFromTargetSpeed           =  QString(valDiv[3]).trimmed().toFloat();
            agent[cIdx]->memory.minimumDistanceToStop                       =  QString(valDiv[4]).trimmed().toFloat();
            agent[cIdx]->memory.actualTargetSpeed                           =  QString(valDiv[5]).trimmed().toFloat();
            agent[cIdx]->memory.actualTargetHeadwayDistance                 =  QString(valDiv[6]).trimmed().toFloat();
            agent[cIdx]->memory.targetSpeed                                 =  QString(valDiv[7]).trimmed().toFloat();
            agent[cIdx]->memory.targetHeadwayDistance                       =  QString(valDiv[8]).trimmed().toFloat();
            agent[cIdx]->memory.targetSpeedByScenario                       =  QString(valDiv[9]).trimmed().toFloat();
            agent[cIdx]->memory.actualTargetHeadwayDistanceByScenario       =  QString(valDiv[10]).trimmed().toFloat();
            agent[cIdx]->memory.targetHeadwayDistanceByScenario             =  QString(valDiv[11]).trimmed().toFloat();
            agent[cIdx]->memory.allowableHeadwayDistDeviation               =  QString(valDiv[12]).trimmed().toFloat();
            agent[cIdx]->memory.targetHeadwayTimeByScenario                 =  QString(valDiv[13]).trimmed().toFloat();
            agent[cIdx]->memory.targetSpeedInsideIntersectionTurnByScenario =  QString(valDiv[14]).trimmed().toFloat();
            agent[cIdx]->memory.startDecel                                  =  QString(valDiv[15]).trimmed().toInt();
            agent[cIdx]->memory.activeBrakeInVelocityControl                = (QString(valDiv[16]).trimmed().toInt() == 1 ? true : false);

            lineNo++;
            cPos = 7;
            continue;
        }
        else if( cIdx >= 0 && cPos == 7 ){

//            qDebug() << " cPos=" << cPos;

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 17 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }

            agent[cIdx]->memory.actualStopAtX            =  QString(valDiv[0]).trimmed().toFloat();
            agent[cIdx]->memory.actualStopAtY            =  QString(valDiv[1]).trimmed().toFloat();
            agent[cIdx]->memory.targetStopAtX            =  QString(valDiv[2]).trimmed().toFloat();
            agent[cIdx]->memory.targetStopAtY            =  QString(valDiv[3]).trimmed().toFloat();
            agent[cIdx]->memory.targetStopAtXByScenario  =  QString(valDiv[4]).trimmed().toFloat();
            agent[cIdx]->memory.targetStopAtYByScenario  =  QString(valDiv[5]).trimmed().toFloat();
            agent[cIdx]->memory.actualStopOnPathID       =  QString(valDiv[6]).trimmed().toInt();
            agent[cIdx]->memory.actualStopOnPathIndex    =  QString(valDiv[7]).trimmed().toInt();
            agent[cIdx]->memory.distToStopAtOnThePath    =  QString(valDiv[8]).trimmed().toFloat();
            agent[cIdx]->memory.distanceToStopPoint      =  QString(valDiv[9]).trimmed().toFloat();
            agent[cIdx]->memory.speedControlState        =  QString(valDiv[10]).trimmed().toInt();
            agent[cIdx]->memory.distanceAdjustLowSpeed   =  QString(valDiv[11]).trimmed().toFloat();
            agent[cIdx]->memory.axSpeedControl           =  QString(valDiv[12]).trimmed().toFloat();
            agent[cIdx]->memory.doHeadwayDistanceControl = (QString(valDiv[13]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.axHeadwayControl         =  QString(valDiv[14]).trimmed().toFloat();
            agent[cIdx]->memory.doStopControl            = (QString(valDiv[15]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.axStopControl            =  QString(valDiv[16]).trimmed().toFloat();
            agent[cIdx]->memory.releaseStopCount         =  QString(valDiv[17]).trimmed().toInt();

            lineNo++;
            cPos = 8;
            continue;
        }
        else if( cIdx >= 0 && cPos == 8 ){

//            qDebug() << " cPos=" << cPos;

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 10 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }

            agent[cIdx]->memory.doSteerControl                               = (QString(valDiv[0]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.lateralDeviationFromTargetPathAtPreviewPoint =  QString(valDiv[1]).trimmed().toFloat();
            agent[cIdx]->memory.previewPointPath                             =  QString(valDiv[2]).trimmed().toInt();
            agent[cIdx]->memory.lateralDeviationFromTargetPath               =  QString(valDiv[3]).trimmed().toFloat();
            agent[cIdx]->memory.steeringControlGainMultiplier                =  QString(valDiv[4]).trimmed().toFloat();
            agent[cIdx]->memory.lateralShiftTarget                           =  QString(valDiv[5]).trimmed().toFloat();
            agent[cIdx]->memory.avoidTarget                                  =  QString(valDiv[6]).trimmed().toInt();
            agent[cIdx]->memory.isChaningLane                                = (QString(valDiv[7]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.speedProfileCount                            =  QString(valDiv[8]).trimmed().toInt();

            // Reset
            agent[cIdx]->memory.lateralShiftTarget = 0.0;

            if( QString(valDiv[9]).isEmpty() == false && QString(valDiv[9]).contains("/") == true ){

                agent[cIdx]->memory.profileTime.clear();

                QStringList profTimeDiv = QString(valDiv[9]).split("/");

                for(int j=0;j<profTimeDiv.size();++j){
                    float val = QString( profTimeDiv[j] ).trimmed().toFloat();
                    agent[cIdx]->memory.profileTime.append( val );
                }
            }

            if( QString(valDiv[10]).isEmpty() == false && QString(valDiv[10]).contains("/") == true ){

                agent[cIdx]->memory.profileSpeed.clear();

                QStringList profSpeedDiv = QString(valDiv[9]).split("/");

                for(int j=0;j<profSpeedDiv.size();++j){
                    float val = QString( profSpeedDiv[j] ).trimmed().toFloat();
                    agent[cIdx]->memory.profileSpeed.append( val );
                }
            }

            lineNo++;
            cPos = 9;
            continue;
        }
        else if( cIdx >= 0 && cPos == 9 ){

//            qDebug() << " cPos=" << cPos;

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 20 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }

            agent[cIdx]->memory.precedingVehicleID           =  QString(valDiv[0]).trimmed().toInt();
            agent[cIdx]->memory.precedingVehicleIDByScenario =  QString(valDiv[1]).trimmed().toInt();
            agent[cIdx]->memory.distanceToPrecedingVehicle   =  QString(valDiv[2]).trimmed().toFloat();
            agent[cIdx]->memory.speedPrecedingVehicle        =  QString(valDiv[3]).trimmed().toFloat();
            agent[cIdx]->memory.axPrecedingVehicle           =  QString(valDiv[4]).trimmed().toFloat();
            agent[cIdx]->memory.precedingObstacle            =  QString(valDiv[5]).trimmed().toInt();
            agent[cIdx]->memory.targetLateralShift           =  QString(valDiv[6]).trimmed().toFloat();
            agent[cIdx]->memory.targetLateralShiftByScenario =  QString(valDiv[7]).trimmed().toFloat();
            agent[cIdx]->memory.distToNearOncomingCP         =  QString(valDiv[8]).trimmed().toFloat();
            agent[cIdx]->memory.distToFatOncomingCP          =  QString(valDiv[9]).trimmed().toFloat();
            agent[cIdx]->memory.shouldWaitOverCrossPoint     = (QString(valDiv[10]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.distToNearestCP              =  QString(valDiv[11]).trimmed().toInt();
            agent[cIdx]->memory.shouldStopAtSignalSL         = (QString(valDiv[12]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.distToYeildStopLine          =  QString(valDiv[13]).trimmed().toFloat();
            agent[cIdx]->memory.shouldYeild                  = (QString(valDiv[14]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.leftCrossIsClear             = (QString(valDiv[15]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.rightCrossIsClear            = (QString(valDiv[16]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.leftCrossCheckCount          =  QString(valDiv[17]).trimmed().toInt();
            agent[cIdx]->memory.rightCrossCheckCount         =  QString(valDiv[18]).trimmed().toInt();
            agent[cIdx]->memory.safetyConfimed               = (QString(valDiv[19]).trimmed().toInt() == 1 ? true : false);
            agent[cIdx]->memory.speedZeroCount               =  QString(valDiv[20]).trimmed().toInt();

            if( valDiv.size() >= 22 ){
                agent[cIdx]->memory.precedingVehicleIndex = QString(valDiv[21]).trimmed().toInt();
            }

            lineNo++;
            cPos = 10;
            continue;
        }
        else if( cIdx >= 0 && cPos == 10 && readLine.startsWith("PO,") == true ){

//            qDebug() << " cPos=" << cPos;

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 42 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }

            int sIndex = QString(valDiv[1]).trimmed().toInt();

            if( agent[cIdx]->memory.perceptedObjects.size() <= sIndex ){
                struct AgentPerception *ap = new struct AgentPerception;
                agent[cIdx]->memory.perceptedObjects.append( ap );
                sIndex = agent[cIdx]->memory.perceptedObjects.size() - 1;
            }

            struct AgentPerception *ap = agent[cIdx]->memory.perceptedObjects[sIndex];

            ap->objectID                         =  QString(valDiv[2]).trimmed().toInt();
            ap->objectType                       =  QString(valDiv[3]).trimmed().toInt();
            ap->vHalfLength                      =  QString(valDiv[4]).trimmed().toFloat();
            ap->vHalfWidth                       =  QString(valDiv[5]).trimmed().toFloat();
            ap->x                                =  QString(valDiv[6]).trimmed().toFloat();
            ap->y                                =  QString(valDiv[7]).trimmed().toFloat();
            ap->yaw                              =  QString(valDiv[8]).trimmed().toFloat();
            ap->V                                =  QString(valDiv[9]).trimmed().toFloat();
            ap->Ax                               =  QString(valDiv[10]).trimmed().toFloat();
            ap->objectPath                       =  QString(valDiv[11]).trimmed().toInt();
            ap->objectTargetNode                 =  QString(valDiv[12]).trimmed().toInt();
            ap->deviationFromObjectPath          =  QString(valDiv[13]).trimmed().toFloat();
            ap->relPosEvaled                     = (QString(valDiv[14]).trimmed().toInt() == 1 ? true : false);
            ap->nearestTargetPath                =  QString(valDiv[15]).trimmed().toInt();
            ap->deviationFromNearestTargetPath   =  QString(valDiv[16]).trimmed().toFloat();
            ap->distanceToObject                 =  QString(valDiv[17]).trimmed().toFloat();
            ap->xOnTargetPath                    =  QString(valDiv[18]).trimmed().toFloat();
            ap->yOnTargetPath                    =  QString(valDiv[19]).trimmed().toFloat();
            ap->innerProductToNearestPathTangent =  QString(valDiv[20]).trimmed().toFloat();
            ap->innerProductToNearestPathNormal  =  QString(valDiv[21]).trimmed().toFloat();
            ap->effectiveHalfWidth               =  QString(valDiv[22]).trimmed().toFloat();
            ap->recognitionLabel                 =  QString(valDiv[23]).trimmed().toInt();
            ap->objPathRecogLabelChecked         =  QString(valDiv[24]).trimmed().toInt();
            ap->myPathRecogLabelChecked          =  QString(valDiv[25]).trimmed().toInt();
            ap->winker                           =  QString(valDiv[26]).trimmed().toInt();
            ap->hasCollisionPoint                = (QString(valDiv[27]).trimmed().toInt() == 1 ? true : false);
            ap->mergingAsCP                      = (QString(valDiv[28]).trimmed().toInt() == 1 ? true : false);
            ap->xCP                              =  QString(valDiv[29]).trimmed().toFloat();
            ap->yCP                              =  QString(valDiv[30]).trimmed().toFloat();
            ap->myDistanceToCP                   =  QString(valDiv[31]).trimmed().toFloat();
            ap->myTimeToCP                       =  QString(valDiv[32]).trimmed().toFloat();
            ap->objectDistanceToCP               =  QString(valDiv[33]).trimmed().toFloat();
            ap->objectTimeToCP                   =  QString(valDiv[34]).trimmed().toFloat();
            ap->CPinNode                         =  QString(valDiv[35]).trimmed().toInt();
            ap->myCPPathIndex                    =  QString(valDiv[36]).trimmed().toInt();
            ap->objCPPathIndex                   =  QString(valDiv[37]).trimmed().toInt();
            ap->objPathCPChecked                 =  QString(valDiv[38]).trimmed().toInt();
            ap->shouldEvalRisk                   = (QString(valDiv[39]).trimmed().toInt() == 1 ? true : false);
            ap->inView                           = (QString(valDiv[40]).trimmed().toInt() == 1 ? true : false);
            ap->noUpdateCount                    =  QString(valDiv[41]).trimmed().toInt();
            ap->isValidData                      = (QString(valDiv[42]).trimmed().toInt() == 1 ? true : false);

            lineNo++;
            continue;
        }
        else if( cIdx >= 0 && cPos == 10 && readLine.startsWith("PS,") == true ){


            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 16 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }

            int psIdx = QString(valDiv[1]).trimmed().toInt();

            if( agent[cIdx]->memory.perceptedSignals.size() <= psIdx ){

                struct TrafficSignalPerception* ts = new struct TrafficSignalPerception;
                agent[cIdx]->memory.perceptedSignals.append( ts );

                psIdx = agent[cIdx]->memory.perceptedSignals.size() - 1;
            }

            struct TrafficSignalPerception* ts = agent[cIdx]->memory.perceptedSignals[psIdx];

            ts->objectID       =  QString(valDiv[2]).trimmed().toInt();

//            qDebug() << " PS, ID=" << ts->objectID;


            ts->objectType     =  QString(valDiv[3]).trimmed().toInt();
            ts->x              =  QString(valDiv[4]).trimmed().toFloat();
            ts->y              =  QString(valDiv[5]).trimmed().toFloat();
            ts->yaw            =  QString(valDiv[6]).trimmed().toFloat();
            ts->relatedNode    =  QString(valDiv[7]).trimmed().toInt();
            ts->signalDisplay  =  QString(valDiv[8]).trimmed().toInt();
            ts->stopLineX      =  QString(valDiv[9]).trimmed().toFloat();
            ts->stopLineY      =  QString(valDiv[10]).trimmed().toFloat();
            ts->distToSL       =  QString(valDiv[11]).trimmed().toFloat();
            ts->SLonPathID     =  QString(valDiv[12]).trimmed().toInt();
            ts->stopPointIndex =  QString(valDiv[13]).trimmed().toInt();
            ts->inView         = (QString(valDiv[14]).trimmed().toInt() == 1 ? true : false);
            ts->noUpdateCount  =  QString(valDiv[15]).trimmed().toInt();
            ts->isValidData    = (QString(valDiv[16]).trimmed().toInt() == 1 ? true : false);

            lineNo++;
            continue;
        }
        else if( readLine.startsWith("TS,") == true ){

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 11 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }

            int idx = QString(valDiv[1]).trimmed().toInt();

            trafficSignal[idx]->id                         = QString(valDiv[2]).trimmed().toInt();

//            qDebug() << " TS, ID=" << trafficSignal[idx]->id;

            trafficSignal[idx]->currentDisplayIndex        = QString(valDiv[3]).trimmed().toInt();
            trafficSignal[idx]->nextUpdateTimeFVal         = QString(valDiv[4]).trimmed().toFloat();
            trafficSignal[idx]->lastUpdateTimeFVal         = QString(valDiv[5]).trimmed().toFloat();
            trafficSignal[idx]->remainingTimeToNextDisplay = QString(valDiv[6]).trimmed().toFloat();
            trafficSignal[idx]->elapsedTimeCurrentDisplay  = QString(valDiv[7]).trimmed().toFloat();
            trafficSignal[idx]->startOffset                = QString(valDiv[8]).trimmed().toInt();
            trafficSignal[idx]->flushState                 = QString(valDiv[9]).trimmed().toInt();
            trafficSignal[idx]->flushTimeCheck             = QString(valDiv[10]).trimmed().toFloat();

            QStringList displayPatternDiv = QString(valDiv[11]).trimmed().split("/");

            for(int j=0;j<trafficSignal[idx]->displayPattern.size();++j){
                delete trafficSignal[idx]->displayPattern[j];
            }
            trafficSignal[idx]->displayPattern.clear();

            for(int j=0;j<displayPatternDiv.size();++j){

                QStringList vals = QString(displayPatternDiv[j]).trimmed().split(":");

                struct SignalDisplayPattern *sdp = new struct SignalDisplayPattern;
                sdp->displayInfo = QString( vals[0] ).trimmed().toInt();
                sdp->duration    = QString( vals[1] ).trimmed().toInt();

                trafficSignal[idx]->displayPattern.append( sdp );
            }

            lineNo++;
            continue;
        }
        else if( readLine.startsWith("OD,") == true ){

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 5 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }

            int idx = QString(valDiv[1]).trimmed().toInt();

            int origNode = QString(valDiv[2]).trimmed().toInt();
            int destNode = QString(valDiv[3]).trimmed().toInt();

//            qDebug() << " OD, O=" << origNode << " D=" << destNode;

            if( idx >= 0 && idx < road->odRoute.size() &&
                    road->odRoute[idx]->originNode == origNode &&
                    road->odRoute[idx]->destinationNode == destNode ){

                road->odRoute[idx]->meanArrivalTime = QString(valDiv[4]).trimmed().toFloat();
                road->odRoute[idx]->NextAppearTime  = QString(valDiv[5]).trimmed().toFloat() - simTimeInSecAtSnapshot;
            }
        }
        else if( readLine.startsWith("PP,") == true ){

            QStringList valDiv = readLine.trimmed().split(",");
            if( valDiv.size() <= 2 ){
                qDebug() << "Data Error: LineNo = " << lineNo << " ; data = " << readLine;
                return;
            }

            int idx = QString(valDiv[1]).trimmed().toInt();
//            qDebug() << " PP, idx=" << idx;

            if( idx >= 0 && idx < road->pedestPaths.size() ){

                road->pedestPaths[idx]->meanArrivalTime = QString(valDiv[2]).trimmed().toFloat();
                road->pedestPaths[idx]->NextAppearTime  = QString(valDiv[3]).trimmed().toFloat() - simTimeInSecAtSnapshot;
            }
        }
    }

    qDebug() << "End of SetRestartData";

    if( DSMode == true ){
        for(int i=0;i<maxAgent;++i){
            if( agent[i]->agentStatus != 1 ){
                continue;
            }
            if( agent[i]->isScenarioObject == true ){
                continue;
            }
            if( agent[i]->isSInterfaceObject == true ){
                continue;
            }
            if( agent[i]->agentKind >= 100 ){
                continue;
            }
            int currentPath = agent[i]->memory.currentTargetPath;
            int pIdx = road->pathId2Index.indexOf( currentPath );
            if( pIdx >= 0 ){
                agent[i]->SetTargetSpeedIndividual( road->paths[pIdx]->speed85pt );
            }
        }
    }

    file.close();

    emit RedrawRequest();
}


