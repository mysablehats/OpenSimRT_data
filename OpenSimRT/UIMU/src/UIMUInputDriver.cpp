/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 */
#include "UIMUInputDriver.h"
#include "Exception.h"
#include <iostream>
#include "SimpleServer.h"

using namespace OpenSimRT;
using namespace SimTK;

UIMUInputDriver::UIMUInputDriver(const int port,
                                                   const double& sendRate)
        :server(port, 1024), rate(sendRate), terminationFlag(false) {}

	// i maybe want to start the server!

UIMUInputDriver::~UIMUInputDriver() { t.join(); }

void UIMUInputDriver::startListening() {
    static auto f = [&]() {
        try {
            for (int i = 0; i < table.getNumRows(); ++i) {
                if (shouldTerminate())
                    THROW_EXCEPTION("File stream terminated.");
                {
                    std::lock_guard<std::mutex> lock(mu);
                    time = table.getIndependentColumn()[i];
                    frame = table.getMatrix()[i];
                    newRow = true;
                }
                cond.notify_one();

                // artificial delay
                std::this_thread::sleep_for(std::chrono::milliseconds(
                        static_cast<int>(1 / rate * 1000)));
            }
            terminationFlag = true;
            cond.notify_one();

        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;

            terminationFlag = true;
            cond.notify_one();
        }
    };
    t = std::thread(f);
}

bool UIMUInputDriver::shouldTerminate() {
    return terminationFlag.load();
}

void UIMUInputDriver::shouldTerminate(bool flag) {
    terminationFlag = flag;
    cond.notify_one();
}

UIMUInputDriver::IMUDataList
UIMUInputDriver::fromVector(const Vector& v) const {
    IMUDataList list;
    UIMUData data;
    int n = UIMUData::size();
    for (int i = 0; i < v.size(); i += n) {
        data.fromVector(v(i, n));
        list.push_back(data);
    }
    return list;
}

UIMUInputDriver::IMUDataList
UIMUInputDriver::getData() const {
    std::unique_lock<std::mutex> lock(mu);
    cond.wait(lock,
              [&]() { return (newRow == true) || terminationFlag.load(); });
    newRow = false;
    return fromVector(frame.getAsVector());
}

std::pair<double, std::vector<UIMUData>> UIMUInputDriver::getFrame() {
    auto temp = getFrameAsVector();
    return std::make_pair(temp.first, fromVector(temp.second));
}

std::pair<double, Vector> UIMUInputDriver::getFrameAsVector() const {
    std::unique_lock<std::mutex> lock(mu);
    cond.wait(lock,
              [&]() { return (newRow == true) || terminationFlag.load(); });
    newRow = false;
    return std::make_pair(time, frame.getAsVector());
}
