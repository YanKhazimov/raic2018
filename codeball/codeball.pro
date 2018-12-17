TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES +=  ../MyStrategy.cpp \
            ../RemoteProcessClient.cpp \
            ../Runner.cpp \
            ../Strategy.cpp \
            ../csimplesocket/ActiveSocket.cpp \
            ../csimplesocket/HTTPActiveSocket.cpp \
            ../csimplesocket/PassiveSocket.cpp \
            ../csimplesocket/SimpleSocket.cpp

HEADERS +=  ../MyStrategy.h \
            ../RemoteProcessClient.h \
            ../Runner.h \
            ../Strategy.h

LIBS += -lws2_32
