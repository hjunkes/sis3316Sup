# sis3316Sup
Epics  Support for Digitizer Struck SIS3316 (VME) (uses asyn)

This is a really first version to make public.
So far only used with MVME6100 under RTEMS-5.
Here there is still a problem with the timing on the VMEbus. There must be a
other arbiter than the VME6100 in the crate. This must be examined still exactly.
If a VMEbus test board (e.g. VMETRO) is in the crate, the error does not occur.
It shows in the "fluttering" of individual bits when reading from the SIS3316. And
when it concerns status bits nothing works any more.
I haven't tried the Flash function yet. I have chosen the delays in sis3316_adc.cpp experimentel ;-)

Some useful queries are still missing (e.g. length of the data arrays). The length must be
specified at "configure" and cannot yet be changed at runtime.

The operator files (bobs) are still missing.

It should also be able to be made to work under vxWorks. The rtems specific parts 
can be found quickly.

in the st.cmd e.g.

#Initialize Struck SIS3316 (16 channel digitizer)
# ..01 as trick to enable Hw Average (other firmware needed), SW1 == 4, SW2 == 1 -> 0x41000000
drvAsynSIS3316Configure("sis0", "0x41000000", "42", "2", "4000", "1", "1")

#asynSetTraceMask ("sis0", -1, 0x3F) set in Port Driver
#asynSetTraceIOMask ("sis0", -1, 0x1)

dbLoadRecords("db/asynRecord.db", "P=PPB:TEST_TEST:SIS0:, R=port, PORT=sis0, ADDR=-1, OMAX=40, IMAX=40")

#Load sis3316 record instance
dbLoadRecords("db/sis3316base.db", "PORT=sis0, P=TEST:XXX:SIS3316:")
dbLoadTemplate("db/sis3316.substitutions", "NAME=TEST:XXX:SIS3316:, HW_PORT=sis0")



