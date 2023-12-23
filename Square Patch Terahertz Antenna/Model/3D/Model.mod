'# MWS Version: Version 2021.1 - Nov 10 2020 - ACIS 30.0.1 -

'# length = um
'# frequency = THz
'# time = ns
'# frequency range: fmin = 0.46 fmax = 1
'# created = '[VERSION]2021.1|30.0.1|20201110[/VERSION]


'@ use template: Antenna - Planar_8.cfg

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
'set the units
With Units
    .Geometry "um"
    .Frequency "THz"
    .Voltage "V"
    .Resistance "Ohm"
    .Inductance "H"
    .TemperatureUnit  "Kelvin"
    .Time "ns"
    .Current "A"
    .Conductance "Siemens"
    .Capacitance "F"
End With

'----------------------------------------------------------------------------

Plot.DrawBox True

With Background
     .Type "Normal"
     .Epsilon "1.0"
     .Mu "1.0"
     .XminSpace "0.0"
     .XmaxSpace "0.0"
     .YminSpace "0.0"
     .YmaxSpace "0.0"
     .ZminSpace "0.0"
     .ZmaxSpace "0.0"
End With

With Boundary
     .Xmin "expanded open"
     .Xmax "expanded open"
     .Ymin "expanded open"
     .Ymax "expanded open"
     .Zmin "expanded open"
     .Zmax "expanded open"
     .Xsymmetry "none"
     .Ysymmetry "none"
     .Zsymmetry "none"
End With

' optimize mesh settings for planar structures

With Mesh
     .MergeThinPECLayerFixpoints "True"
     .RatioLimit "20"
     .AutomeshRefineAtPecLines "True", "6"
     .FPBAAvoidNonRegUnite "True"
     .ConsiderSpaceForLowerMeshLimit "False"
     .MinimumStepNumber "5"
     .AnisotropicCurvatureRefinement "True"
     .AnisotropicCurvatureRefinementFSM "True"
End With

With MeshSettings
     .SetMeshType "Hex"
     .Set "RatioLimitGeometry", "20"
     .Set "EdgeRefinementOn", "1"
     .Set "EdgeRefinementRatio", "6"
End With

With MeshSettings
     .SetMeshType "HexTLM"
     .Set "RatioLimitGeometry", "20"
End With

With MeshSettings
     .SetMeshType "Tet"
     .Set "VolMeshGradation", "1.5"
     .Set "SrfMeshGradation", "1.5"
End With

' change mesh adaption scheme to energy
' 		(planar structures tend to store high energy
'     	 locally at edges rather than globally in volume)

MeshAdaption3D.SetAdaptionStrategy "Energy"

' switch on FD-TET setting for accurate farfields

FDSolver.ExtrudeOpenBC "True"

PostProcess1D.ActivateOperation "vswr", "true"
PostProcess1D.ActivateOperation "yz-matrices", "true"

With FarfieldPlot
	.ClearCuts ' lateral=phi, polar=theta
	.AddCut "lateral", "0", "1"
	.AddCut "lateral", "90", "1"
	.AddCut "polar", "90", "1"
End With

'----------------------------------------------------------------------------

With MeshSettings
     .SetMeshType "Hex"
     .Set "Version", 1%
End With

With Mesh
     .MeshType "PBA"
End With

'set the solver type
ChangeSolverType("HF Time Domain")

'----------------------------------------------------------------------------

'@ activate local coordinates

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
WCS.ActivateWCS "local"

'@ define material: Silver

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Material
     .Reset
     .Name "Silver"
     .Folder ""
     .FrqType "static"
     .Type "Normal"
     .SetMaterialUnit "Hz", "mm"
     .Epsilon "1"
     .Mu "1.0"
     .Kappa "6.3012e007"
     .TanD "0.0"
     .TanDFreq "0.0"
     .TanDGiven "False"
     .TanDModel "ConstTanD"
     .KappaM "0"
     .TanDM "0.0"
     .TanDMFreq "0.0"
     .TanDMGiven "False"
     .TanDMModel "ConstTanD"
     .DispModelEps "None"
     .DispModelMu "None"
     .DispersiveFittingSchemeEps "General 1st"
     .DispersiveFittingSchemeMu "General 1st"
     .UseGeneralDispersionEps "False"
     .UseGeneralDispersionMu "False"
     .FrqType "all"
     .Type "Lossy metal"
     .MaterialUnit "Frequency", "GHz"
     .MaterialUnit "Geometry", "mm"
     .MaterialUnit "Time", "s"
     .MaterialUnit "Temperature", "Kelvin"
     .Mu "1.0"
     .Sigma "6.3012e007"
     .Rho "10500.0"
     .ThermalType "Normal"
     .ThermalConductivity "429"
     .SpecificHeat "230", "J/K/kg"
     .MetabolicRate "0"
     .BloodFlow "0"
     .VoxelConvection "0"
     .MechanicsType "Isotropic"
     .YoungsModulus "76"
     .PoissonsRatio "0.37"
     .ThermalExpansionRate "20"
     .ReferenceCoordSystem "Global"
     .CoordSystemType "Cartesian"
     .NLAnisotropy "False"
     .NLAStackingFactor "1"
     .NLADirectionX "1"
     .NLADirectionY "0"
     .NLADirectionZ "0"
     .Colour "1", "1", "0"
     .Wireframe "False"
     .Reflection "False"
     .Allowoutline "True"
     .Transparentoutline "False"
     .Transparency "0"
     .Create
End With

'@ new component: component1

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Component.New "component1"

'@ define brick: component1:ground

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Brick
     .Reset 
     .Name "ground" 
     .Component "component1" 
     .Material "Silver" 
     .Xrange "-250", "250" 
     .Yrange "-200", "200" 
     .Zrange "0", "-10" 
     .Create
End With

'@ define material: Arlon AD 600 (lossy)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Material
     .Reset
     .Name "Arlon AD 600 (lossy)"
     .Folder ""
     .FrqType "all"
     .Type "Normal"
     .SetMaterialUnit "GHz", "mm"
     .Epsilon "6.15"
     .Mu "1.0"
     .Kappa "0.0"
     .TanD "0.003"
     .TanDFreq "10.0"
     .TanDGiven "True"
     .TanDModel "ConstTanD"
     .KappaM "0.0"
     .TanDM "0.0"
     .TanDMFreq "0.0"
     .TanDMGiven "False"
     .TanDMModel "ConstKappa"
     .DispModelEps "None"
     .DispModelMu "None"
     .DispersiveFittingSchemeEps "General 1st"
     .DispersiveFittingSchemeMu "General 1st"
     .UseGeneralDispersionEps "False"
     .UseGeneralDispersionMu "False"
     .Rho "0.0"
     .ThermalType "Normal"
     .ThermalConductivity "0.46"
     .SetActiveMaterial "all"
     .Colour "0.94", "0.82", "0.76"
     .Wireframe "False"
     .Transparency "0"
     .Create
End With

'@ define brick: component1:Substrate

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Brick
     .Reset 
     .Name "Substrate" 
     .Component "component1" 
     .Material "Arlon AD 600 (lossy)" 
     .Xrange "-250", "250" 
     .Yrange "-200", "200" 
     .Zrange "0", "100" 
     .Create
End With

'@ pick face

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Pick.PickFaceFromId "component1:Substrate", "1"

'@ align wcs with face

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
WCS.AlignWCSWithSelected "Face"

'@ define brick: component1:Patch

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Brick
     .Reset 
     .Name "Patch" 
     .Component "component1" 
     .Material "Arlon AD 600 (lossy)" 
     .Xrange "-120", "120" 
     .Yrange "-95", "95" 
     .Zrange "0", "5" 
     .Create
End With

'@ change material: component1:Patch to: Silver

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Solid.ChangeMaterial "component1:Patch", "Silver"

'@ define brick: component1:Feed

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Brick
     .Reset 
     .Name "Feed" 
     .Component "component1" 
     .Material "Arlon AD 600 (lossy)" 
     .Xrange "-25", "25" 
     .Yrange "-200", "0" 
     .Zrange "0", "0" 
     .Create
End With

'@ delete shape: component1:Feed

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Solid.Delete "component1:Feed"

'@ define brick: component1:Feed

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Brick
     .Reset 
     .Name "Feed" 
     .Component "component1" 
     .Material "Silver" 
     .Xrange "-50/2", "50/2" 
     .Yrange "-400/2", "-190/2" 
     .Zrange "0", "5" 
     .Create
End With

'@ boolean add shapes: component1:Patch, component1:Feed

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Solid.Add "component1:Patch", "component1:Feed"

'@ pick face

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Pick.PickFaceFromId "component1:Patch", "3"

'@ define port: 1

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Port 
     .Reset 
     .PortNumber "1" 
     .Label ""
     .Folder ""
     .NumberOfModes "1"
     .AdjustPolarization "False"
     .PolarizationAngle "0.0"
     .ReferencePlaneDistance "0"
     .TextSize "50"
     .TextMaxLimit "0"
     .Coordinates "Picks"
     .Orientation "positive"
     .PortOnBound "True"
     .ClipPickedPortToBound "False"
     .Xrange "-25", "25"
     .Yrange "-200", "-200"
     .Zrange "100", "105"
     .XrangeAdd "0.0", "0.0"
     .YrangeAdd "0.0", "0.0"
     .ZrangeAdd "0.0", "0.0"
     .SingleEnded "False"
     .WaveguideMonitor "False"
     .Create 
End With

'@ define frequency range

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Solver.FrequencyRange "0.05", "0.15"

'@ define time domain solver parameters

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Mesh.SetCreator "High Frequency" 

With Solver 
     .Method "Hexahedral"
     .CalculationType "TD-S"
     .StimulationPort "All"
     .StimulationMode "All"
     .SteadyStateLimit "-40"
     .MeshAdaption "False"
     .AutoNormImpedance "False"
     .NormingImpedance "50"
     .CalculateModesOnly "False"
     .SParaSymmetry "False"
     .StoreTDResultsInCache  "False"
     .FullDeembedding "False"
     .SuperimposePLWExcitation "False"
     .UseSensitivityAnalysis "False"
End With

'@ set PBA version

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Discretizer.PBAVersion "2020111021"

'@ delete port: port1

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Port.Delete "1"

'@ define port: 1

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Port 
     .Reset 
     .PortNumber "1" 
     .Label ""
     .Folder ""
     .NumberOfModes "1"
     .AdjustPolarization "False"
     .PolarizationAngle "0.0"
     .ReferencePlaneDistance "0"
     .TextSize "50"
     .TextMaxLimit "1"
     .Coordinates "Full"
     .Orientation "ymin"
     .PortOnBound "True"
     .ClipPickedPortToBound "False"
     .Xrange "-999.481145", "999.481145"
     .Yrange "-949.481145", "-949.481145"
     .Zrange "-759.481145", "854.481145"
     .XrangeAdd "0.0", "0.0"
     .YrangeAdd "0.0", "0.0"
     .ZrangeAdd "0.0", "0.0"
     .SingleEnded "False"
     .WaveguideMonitor "False"
     .Create 
End With

'@ define frequency range

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Solver.FrequencyRange "0.1", "1"

'@ activate global coordinates

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
WCS.ActivateWCS "global"

'@ activate local coordinates

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
WCS.ActivateWCS "local"

'@ activate global coordinates

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
WCS.ActivateWCS "global"

'@ define monitor: e-field (f=0.49)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "e-field (f=0.49)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Efield" 
     .MonitorValue "0.49" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-250", "250", "-200", "200", "-10", "105" 
     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
     .SetSubvolumeInflateWithOffset "False" 
     .Create 
End With

'@ define monitor: h-field (f=0.49)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "h-field (f=0.49)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Hfield" 
     .MonitorValue "0.49" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-250", "250", "-200", "200", "-10", "105" 
     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
     .SetSubvolumeInflateWithOffset "False" 
     .Create 
End With

'@ define farfield monitor: farfield (f=0.49)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "farfield (f=0.49)" 
     .Domain "Frequency" 
     .FieldType "Farfield" 
     .MonitorValue "0.49" 
     .ExportFarfieldSource "False" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-250", "250", "-200", "200", "-10", "105" 
     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
     .SetSubvolumeInflateWithOffset "False" 
     .SetSubvolumeOffsetType "FractionOfWavelength" 
     .EnableNearfieldCalculation "True" 
     .Create 
End With

'@ define fieldsource monitor: field-source (f=0.49)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "field-source (f=0.49)" 
     .Domain "Frequency" 
     .FieldType "Fieldsource" 
     .Samples "1" 
     .MonitorValueRange "0.49", "0.49" 
     .InvertOrientation "False" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-250", "250", "-200", "200", "-10", "105" 
     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
     .SetSubvolumeInflateWithOffset "False" 
     .SetSubvolumeOffsetType "FractionOfWavelength" 
     .Create 
End With

'@ define monitor: surface-current (f=0.49)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "surface-current (f=0.49)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Surfacecurrent" 
     .MonitorValue "0.49" 
     .Create 
End With

'@ define monitor: power (f=0.49)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "power (f=0.49)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Powerflow" 
     .MonitorValue "0.49" 
     .Create 
End With

'@ define monitor: current-density (f=0.49)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "current-density (f=0.49)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Current" 
     .MonitorValue "0.49" 
     .Create 
End With

'@ define monitor: loss (f=0.49)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "loss (f=0.49)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Powerloss" 
     .MonitorValue "0.49" 
     .Create 
End With

'@ define monitor: e-energy (f=0.49)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "e-energy (f=0.49)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Eenergy" 
     .MonitorValue "0.49" 
     .Create 
End With

'@ define monitor: h-energy (f=0.49)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "h-energy (f=0.49)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Henergy" 
     .MonitorValue "0.49" 
     .Create 
End With

'@ define frequency range

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
Solver.FrequencyRange "0.46", "1"

'@ farfield plot options

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With FarfieldPlot 
     .Plottype "3D" 
     .Vary "angle1" 
     .Theta "90" 
     .Phi "90" 
     .Step "5" 
     .Step2 "5" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "True" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .AspectRatio "Free" 
     .ShowGridlines "True" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "Directivity" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .IncludeUnitCellSidewalls "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .SetMaxReferenceMode "abs" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+00 
     .Origin "bbox" 
     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+00" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+01" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .ClearCuts 
     .AddCut "lateral", "0", "1"  
     .AddCut "lateral", "90", "1"  
     .AddCut "polar", "90", "1"  

     .StoreSettings
End With

'@ define monitor: e-field (f=0.5)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "e-field (f=0.5)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Efield" 
     .MonitorValue "0.50" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-250", "250", "-200", "200", "-10", "105" 
     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
     .SetSubvolumeInflateWithOffset "False" 
     .Create 
End With

'@ define monitor: h-field (f=0.5)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "h-field (f=0.5)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Hfield" 
     .MonitorValue "0.50" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-250", "250", "-200", "200", "-10", "105" 
     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
     .SetSubvolumeInflateWithOffset "False" 
     .Create 
End With

'@ define farfield monitor: farfield (f=0.5)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "farfield (f=0.5)" 
     .Domain "Frequency" 
     .FieldType "Farfield" 
     .MonitorValue "0.50" 
     .ExportFarfieldSource "False" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-250", "250", "-200", "200", "-10", "105" 
     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
     .SetSubvolumeInflateWithOffset "False" 
     .SetSubvolumeOffsetType "FractionOfWavelength" 
     .EnableNearfieldCalculation "True" 
     .Create 
End With

'@ define monitor: surface-current (f=0.5)

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With Monitor 
     .Reset 
     .Name "surface-current (f=0.5)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Surfacecurrent" 
     .MonitorValue "0.50" 
     .Create 
End With

'@ farfield plot options

'[VERSION]2021.1|30.0.1|20201110[/VERSION]
With FarfieldPlot 
     .Plottype "Polar" 
     .Vary "angle2" 
     .Theta "90" 
     .Phi "90" 
     .Step "1" 
     .Step2 "1" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "0.5" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "True" 
     .ShowStructureProfile "True" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "True" 
     .AspectRatio "Free" 
     .ShowGridlines "True" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "Directivity" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .IncludeUnitCellSidewalls "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .SetMaxReferenceMode "abs" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "unknown" 
     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Linear" 
     .SlantAngle 0.000000e+00 
     .Origin "bbox" 
     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+00" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+01" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .ClearCuts 
     .AddCut "lateral", "0", "1"  
     .AddCut "lateral", "90", "1"  
     .AddCut "polar", "90", "1"  

     .StoreSettings
End With

