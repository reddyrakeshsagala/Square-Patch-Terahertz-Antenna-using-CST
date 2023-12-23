<?xml version="1.0" encoding="UTF-8"?>
<MetaResultFile version="20200629" creator="Solver HFTD - Field 3DFD Monitor">
  <MetaGeometryFile filename="model.gex" lod="1"/>
  <SimulationProperties fieldname="surface current (f=0.49) [1]" fieldtype="Surface current" frequency="0.49000000953674316" encoded_unit="&amp;U:A^1.:m^-1" fieldscaling="PEAK" dB_Amplitude="20"/>
  <ResultDataType vector="1" complex="1" timedomain="0"/>
  <SimulationDomain min="-352.66864 -302.66864 -112.668648" max="352.66864 302.66864 207.668655"/>
  <PlotSettings Plot="2" ignore_symmetry="0" deformation="0" enforce_culling="0" default_arrow_type="ARROWS"/>
  <Source type="SOLVER"/>
  <SpecialMaterials>
    <Background type="NORMAL"/>
    <Material name="Silver" type="FIELDFREE"/>
  </SpecialMaterials>
  <AuxGeometryFile/>
  <AuxResultFile/>
  <FieldFreeNodes/>
  <SurfaceFieldCoefficients/>
  <UnitCell/>
  <SubVolume/>
  <Units/>
  <ResultGroups num_steps="1" transformation="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1">
    <SharedDataWith treepath="2D/3D Results\H-Field\h-field (f=0.49) [1]" rexname="h-field (f=0.49)_1,1_m3d.rex"/>
    <Frame index="0">
      <FieldResultFile filename="h-field (f=0.49)_1,1.m3d" type="m3d"/>
    </Frame>
  </ResultGroups>
</MetaResultFile>
