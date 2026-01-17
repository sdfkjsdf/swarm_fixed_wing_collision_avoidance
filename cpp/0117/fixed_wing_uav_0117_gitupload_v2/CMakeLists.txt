cmake_minimum_required(VERSION 3.10)

# 프로젝트 이름 설정
project(UAV_Practice CXX C)

# C++ 표준 설정 (C++17 권장)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC(Visual Studio) 컴파일러 사용 시 인코딩 경고(C4819) 해결을 위한 옵션 추가
if(MSVC)
    add_compile_options(/utf-8)
endif()

# [핵심] 헤더 파일 경로 설정
# include_directories 명령어를 사용하면 컴파일러에게 "이 경로에서도 헤더 파일을 찾아라"고 알려줍니다.
# ${CMAKE_SOURCE_DIR}은 이 CMakeLists.txt가 있는 최상위 폴더를 의미합니다.
# 이렇게 설정하면 소스 코드에서 #include <matrix/math.hpp> 처럼 꺾쇠(<>)를 사용할 수 있습니다.
include_directories(${CMAKE_SOURCE_DIR})


add_library(Fixed_wing_uav_model 
            fixed_wing_aircraft_model/FixedwingSpec.c 
            fixed_wing_aircraft_model/FixedwingDynamics.cpp
            )

add_library(Controller_library 
            controller/GainParameter.c 
            controller/NdiParameter.c
            controller/FlightEnvelope.cpp
            controller/PitchSetpoint.cpp
            controller/RollratePitchrateSetpoint.cpp
            controller/BodyAxisRateSetpoint.cpp
            controller/MainController.cpp
            controller/ThrustSetpoint.cpp
            )

add_library(Guidance_library
            formation_guidance/FlockingGuidance.cpp
            formation_guidance/FlockingParameter.c
            formation_guidance/LeaderFollowerGuidance.cpp
            formation_guidance/LeaderFollowerParameter.c
            formation_guidance/MainFormationGuidance.cpp)

add_library(Agent_library
            agent/Agent.cpp)



# 실행 파일 생성
# practice 폴더 안의 practice.cpp를 빌드하여 practice라는 실행 파일을 만듭니다.
add_executable(practice practice/practice.cpp)


target_link_libraries(practice Fixed_wing_uav_model)
target_link_libraries(practice Controller_library)
target_link_libraries(practice Guidance_library)
target_link_libraries(practice Agent_library)





### 라이브러리 형식으로 모듈화를 묶는 방법


# add_library(라이브러리 이름
#     라이브러리 구성 요소가 있는 폴더명 /cpp 파일 (소스 파일)
#     라이브러리 구성 요소가 있는 폴더명 /cpp 파일을 구성하는 또다른 cpp 파일 (소스파일)
# )
    #

# # 2. [실행] 메인 파일로 실행 파일 생성
# add_executable(내가 만들 실행파일의 이름  main cpp가 있는 폴더명 /main.cpp)

# # 3. [연결] 실행 파일에 라이브러리 붙이기
# target_link_libraries(내가 만들 실행파일의 이름 라이브러리 이름)

## 라이브러리 형식으로 사용 시 실절적으로 main 문 주소 지정의 과정

# include < #include <라이브러리_구성요소의_헤더가_있는_폴더명/헤더파일.hpp> 







### 각종 오류에 대한 해결방법 정리본 ##

# 1. 일단 지금  "C/C++ Runner" 라는 확장 프로그램 을 무조건 삭제하고 진행을 해야 함
# 2. CMake 프로젝트 설정 (최초 1회 Ctrl + Shift + P를 눌러 명령어 창 -> 컴파일러를 선택하라고 나오면 설치된 컴파일러(예: GCC, Clang, 또는 Visual Studio Community ...)를 선택)
# 3. VS Code 하단 상태 표시줄에 "Build" 버튼이 생겼을 것입니다. (또는 F7 키) , 이걸 클릭하거나, Ctrl + Shift + P -> CMake: Build 를 실행합니다.
# 실행만 할 때: Ctrl + F5 (또는 하단의 ▶ 플레이 버튼)
# 디버깅 할 때 (F5처럼): Ctrl + F5 옆에 있는 벌레 모양 아이콘이나, 명령어 창에서 CMake: Debug 선택





