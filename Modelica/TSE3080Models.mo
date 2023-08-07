within ;
package TSE3080Models
  extends Modelica.Icons.Package;
  package Transformer "Examples used during the transformer lectures"
    extends Modelica.Icons.Package;
    model SimpleMagneticCircuit
      extends Modelica.Magnetic.FluxTubes.Examples.BasicExamples.SaturatedInductor;
      annotation (experiment(StopTime=0.1, Tolerance=1e-07));
    end SimpleMagneticCircuit;

     model CompareSaturationEffects
      "Inductor with saturation in the ferromagnetic core"
      extends Modelica.Magnetic.FluxTubes.Examples.BasicExamples.SaturatedInductor;

      Modelica.Magnetic.FluxTubes.Basic.Ground ground_m1
                                                        annotation (Placement(
            transformation(extent={{50,-94},{70,-74}}, rotation=0)));
      Modelica.Electrical.Analog.Sources.SineVoltage source1(
        f=50,
        phase=Modelica.Constants.pi/2,
        V=230*sqrt(2)) "Voltage applied to inductor" annotation (Placement(transformation(
            origin={-80,-54},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Electrical.Analog.Basic.Resistor r1(
                                                  R=7.5)
        "Inductor coil resistance" annotation (Placement(transformation(extent={{-61,-54},
                {-41,-34}},          rotation=0)));
      Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverter coil1(
                                                                      N=600, i(
            fixed=true)) "Inductor coil" annotation (Placement(transformation(
              extent={{-30,-64},{-10,-44}},
                                         rotation=0)));
      Modelica.Magnetic.FluxTubes.Basic.ConstantReluctance r_mLeak1(R_m=1.2e6)
        "Constant leakage reluctance" annotation (Placement(transformation(
            origin={10,-54},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Magnetic.FluxTubes.Shapes.FixedShape.Cuboid r_mAirPar1(
        a=0.025,
        b=0.025,
        nonLinearPermeability=false,
        mu_rConst=1,
        l=0.0001)
        "Reluctance of small parasitic air gap (ferromagnetic core packeted from single sheets)"
        annotation (Placement(transformation(extent={{26,-54},{46,-34}},
                                                                       rotation=
               0)));
      Modelica.Magnetic.FluxTubes.Shapes.FixedShape.Cuboid r_mFe1(
        mu_rConst=1000,
        a=0.025,
        b=0.025,
        l=4*0.065,
        material=
            Modelica.Magnetic.FluxTubes.Material.SoftMagnetic.ElectricSheet.M350_50A(),
        B(start=0),
        nonLinearPermeability=false)
        "Reluctance of ferromagnetic inductor core"             annotation (
          Placement(transformation(
            origin={60,-54},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Electrical.Analog.Basic.Ground ground1
                                                     annotation (Placement(
            transformation(extent={{-90,-94},{-70,-74}}, rotation=0)));
     equation
      connect(source1.p, r1.p)
        annotation (Line(points={{-80,-44},{-61,-44}},
                                                   color={0,0,255}));
      connect(r1.n, coil1.p)
        annotation (Line(points={{-41,-44},{-30,-44},{-30,-44}},
                                                            color={0,0,255}));
      connect(source1.n, coil1.n) annotation (Line(points={{-80,-64},{-30,-64},{-30,-64}},
                          color={0,0,255}));
      connect(coil1.port_p, r_mLeak1.port_p)
        annotation (Line(points={{-10,-44},{-10,-44},{10,-44}},
                                                           color={255,127,0}));
      connect(r_mLeak1.port_p, r_mAirPar1.port_p)
        annotation (Line(points={{10,-44},{26,-44}},
                                                 color={255,127,0}));
      connect(r_mAirPar1.port_n, r_mFe1.port_p)
        annotation (Line(points={{46,-44},{54,-44},{60,-44}},
                                                        color={255,127,0}));
      connect(r_mFe1.port_n, r_mLeak1.port_n) annotation (Line(points={{60,-64},{47.5,
              -64},{35,-64},{10,-64}},       color={255,127,0}));
      connect(r_mFe1.port_n, coil1.port_n) annotation (Line(points={{60,-64},{-10,-64},{-10,-64}},
                                   color={255,127,0}));
      connect(ground1.p, source1.n) annotation (Line(
          points={{-80,-74},{-80,-64}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ground_m1.port, r_mFe1.port_n) annotation (Line(
          points={{60,-74},{60,-64}},
          color={255,127,0},
          smooth=Smooth.None));
      annotation (experiment(StopTime=0.1, Tolerance=1e-007), Documentation(
            info="<html>
<p>
This model demonstrates the effects of non-linear magnetisation characteristics of soft magnetic materials (hysteresis neglected). A sinusoidal voltage is applied to an inductor with a closed ferromagnetic core of rectangular shape. Set the <b>tolerance</b> to <b>1e-7</b>, <b>simulate for 0.1 s</b> and plot for example:
</p>

<pre>
    coil.i vs. time           // non-harmonic current due to saturation of the core material
    r_mFe.mu_r vs. r_mFe.B    // relative permeability vs. flux density inside core
    r_mFe.B vs. r_mFe.H       // magnetisation curve B(H); hysteresis neglected
</pre>

<p>
The magnetisation characteristics of the flux tube element representing the ferromagnetic core can easily be changed from simplified linear behaviour (nonLinearPermeability set to false and R_mFe.mu_rConst set to a positive value, preferably mu_rConst >> 1) to non-linear behaviour (e.g., selection of one of the electric sheets in <a href=\"modelica://Modelica.Magnetic.FluxTubes.Material.SoftMagnetic\">Material.SoftMagnetic</a> with nonLinearPermeability set to true). This enables for convenient inital design of magnetic circuits with linear material characteristics prior to simulation with non-linear behaviour.
</p>

<h4>Note</h4>

<p>
If the supply voltage has a zero-crossing when applied to the inductor at time t=0 (i.e., source.phase set to zero instead of &pi;/2), then the inrush current that is typical for switching of inductive loads can be observed.
</p>
</html>"),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics={Text(
              extent={{-40,40},{20,34}},
              lineColor={0,0,255},
              textString="Saturated"), Text(
              extent={{-40,-24},{20,-30}},
              lineColor={0,0,255},
              textString="Non-saturated")}));
     end CompareSaturationEffects;

    model TransformerTestbench "Transformer Testbench"
      extends Modelica.Icons.Example;
      parameter Modelica.Units.SI.Resistance RL[3]=fill(1/3, 3) "Load resistance";
      Modelica.Electrical.Polyphase.Sources.SineVoltage source(f=fill(50, 3), V=fill(sqrt(2/3)*100, 3)) annotation (Placement(transformation(
            origin={-90,-10},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.Polyphase.Basic.Star starS annotation (Placement(transformation(
            origin={-90,-40},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Electrical.Analog.Basic.Ground groundS
        annotation (Placement(transformation(extent={{-100,-80},{-80,-60}},
              rotation=0)));
      Modelica.Electrical.Machines.Sensors.ElectricalPowerSensor electricalPowerSensorS
        annotation (Placement(transformation(extent={{-90,0},{-70,20}},
              rotation=0)));
      Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensorS
        annotation (Placement(transformation(extent={{-60,20},{-40,0}},
              rotation=0)));
      Modelica.Electrical.Machines.Sensors.VoltageQuasiRMSSensor voltageQuasiRMSSensorS
        annotation (Placement(transformation(
            origin={-50,-30},
            extent={{-10,10},{10,-10}},
            rotation=180)));
      Modelica.Electrical.Polyphase.Basic.Delta deltaS annotation (Placement(transformation(
            origin={-50,-10},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      Modelica.Electrical.Analog.Basic.Resistor earth(R=1e6)
        annotation (Placement(transformation(
            origin={0,-40},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.Analog.Basic.Ground groundT
        annotation (Placement(transformation(extent={{-10,-80},{10,-60}},
              rotation=0)));
      Modelica.Electrical.Machines.Sensors.VoltageQuasiRMSSensor voltageRMSSensorL
        annotation (Placement(transformation(
            origin={50,-30},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      Modelica.Electrical.Polyphase.Basic.Delta deltaL annotation (Placement(transformation(
            origin={50,-10},
            extent={{-10,10},{10,-10}},
            rotation=180)));
      Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensorL
        annotation (Placement(transformation(extent={{40,20},{60,0}}, rotation=
                0)));
      Modelica.Electrical.Machines.Sensors.ElectricalPowerSensor electricalPowerSensorL
        annotation (Placement(transformation(extent={{70,0},{90,20}}, rotation=
                0)));
      Modelica.Electrical.Polyphase.Basic.Resistor load(R=RL) annotation (Placement(transformation(
            origin={90,-10},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.Polyphase.Basic.Star starL annotation (Placement(transformation(
            origin={90,-40},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.Analog.Basic.Ground groundL
        annotation (Placement(transformation(extent={{80,-80},{100,-60}},
              rotation=0)));
      parameter Modelica.Electrical.Machines.Utilities.TransformerData transformerData(
        C1=Modelica.Utilities.Strings.substring(
                transformer.VectorGroup,
                1,
                1),
        C2=Modelica.Utilities.Strings.substring(
                transformer.VectorGroup,
                2,
                2),
        f=50,
        V1=100,
        V2=100,
        SNominal=30E3,
        v_sc=0.05,
        P_sc=300) annotation (Placement(transformation(extent={{-10,40},{10,60}},
              rotation=0)));
      Modelica.Electrical.Machines.BasicMachines.Transformers.Yy.Yy00 transformer(
        n=transformerData.n,
        R1=transformerData.R1,
        L1sigma=transformerData.L1sigma,
        R2=transformerData.R2,
        L2sigma=transformerData.L2sigma,
        alpha20_1(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
        alpha20_2(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
        T1Ref=293.15,
        T2Ref=293.15,
        T1Operational=293.15,
        T2Operational=293.15) annotation (Placement(transformation(extent={{-20,
                -10},{20,30}}, rotation=0)));

    initial equation
      transformer.i2[1:2]=zeros(2);
    equation
      connect(starS.pin_n, groundS.p)
        annotation (Line(points={{-90,-50},{-90,-60}}, color={0,0,255}));
      connect(source.plug_n, starS.plug_p)
        annotation (Line(points={{-90,-20},{-90,-30}}, color={0,0,255}));
      connect(starL.pin_n, groundL.p)
        annotation (Line(points={{90,-50},{90,-60}}, color={0,0,255}));
      connect(load.plug_n, starL.plug_p)
        annotation (Line(points={{90,-20},{90,-30}}, color={0,0,255}));
      connect(earth.n, groundT.p)
        annotation (Line(points={{0,-50},{0,-50},{0,-60}},             color={0,
              0,255}));
      connect(electricalPowerSensorS.plug_nv, starS.plug_p)
        annotation (Line(points={{-80,0},{-80,-30},{-90,-30}}, color={0,0,255}));
      connect(source.plug_p, electricalPowerSensorS.plug_p)
        annotation (Line(points={{-90,0},{-90,10}}, color={0,0,255}));
      connect(electricalPowerSensorS.plug_ni, currentQuasiRMSSensorS.plug_p)
        annotation (Line(points={{-70,10},{-60,10}}, color={0,0,255}));
      connect(currentQuasiRMSSensorL.plug_n, electricalPowerSensorL.plug_p)
        annotation (Line(points={{60,10},{70,10}}, color={0,0,255}));
      connect(electricalPowerSensorL.plug_ni, load.plug_p)
        annotation (Line(points={{90,10},{90,0}}, color={0,0,255}));
      connect(electricalPowerSensorL.plug_nv, starL.plug_p)
        annotation (Line(points={{80,0},{80,-30},{90,-30}}, color={0,0,255}));
      connect(currentQuasiRMSSensorS.plug_n, transformer.plug1)
                                                           annotation (Line(
          points={{-40,10},{-20,10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transformer.plug2, currentQuasiRMSSensorL.plug_p)
                                                           annotation (Line(
          points={{20,10},{40,10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(deltaS.plug_p, voltageQuasiRMSSensorS.plug_n) annotation (Line(
          points={{-60,-10},{-60,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(deltaS.plug_n, voltageQuasiRMSSensorS.plug_p) annotation (Line(
          points={{-40,-10},{-40,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(currentQuasiRMSSensorS.plug_n, deltaS.plug_n) annotation (Line(
          points={{-40,10},{-40,-10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(currentQuasiRMSSensorL.plug_p, deltaL.plug_n) annotation (Line(
          points={{40,10},{40,-10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(deltaL.plug_n, voltageRMSSensorL.plug_p) annotation (Line(
          points={{40,-10},{40,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(deltaL.plug_p, voltageRMSSensorL.plug_n) annotation (Line(
          points={{60,-10},{60,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transformer.starpoint1, earth.p) annotation (Line(points={{-10,
              -10},{-10,-26},{0,-26},{0,-30}}, color={0,0,255}));
      annotation (Documentation(info="<HTML>
<h4>Transformer testbench:</h4>
<p>
You may choose different connections as well as vary the load (even not symmetrical).
</p>
<p>
<b>Please pay attention</b> to proper grounding of the primary and secondary part of the whole circuit.<br>
The primary and secondary starpoint are available as connectors, if the connection is not delta (D or d).
</p>
<p>
In some cases it may be necessary to ground the transformer's starpoint even though the source's or load's starpoint are grounded:
</p>
<ul>
<li>Yy ... Grounding of transformer's primary or secondary starpoint with reasonable high earthing resistance is necessary.</li>
<li>Yd ... No grounding necessary.</li>
<li>Yz ... Grounding of transformer's primary starpoint with reasonable high earthing resistance is necessary.</li>
<li>Dy ... No grounding necessary.</li>
<li>Dd ... No grounding necessary.</li>
<li>Dz ... No grounding necessary.</li>
</ul>
</HTML>"),        experiment(StopTime=0.1, Interval=0.001),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}})));
    end TransformerTestbench;

    model AsymmetricalLoad "AsymmetricalLoad"
      extends Modelica.Icons.Example;
      parameter Modelica.Units.SI.Resistance RL=1 "Load resistance";
      Modelica.Electrical.Polyphase.Sources.SineVoltage source(f=fill(50, 3), V=fill(sqrt(2/3)*100, 3)) annotation (Placement(transformation(
            origin={-90,-10},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.Polyphase.Basic.Star starS annotation (Placement(transformation(
            origin={-90,-40},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Electrical.Analog.Basic.Ground groundS annotation (Placement(
            transformation(extent={{-100,-80},{-80,-60}})));
      Modelica.Electrical.Polyphase.Sensors.CurrentSensor currentSensorS annotation (Placement(transformation(extent={{-60,20},{-40,0}})));
      Modelica.Electrical.Analog.Basic.Ground groundL annotation (Placement(
            transformation(extent={{0,-80},{20,-60}})));
      parameter
        Modelica.Electrical.Machines.Utilities.TransformerData transformerData(
        C1=Modelica.Utilities.Strings.substring(
                transformer.VectorGroup,
                1,
                1),
        C2=Modelica.Utilities.Strings.substring(
                transformer.VectorGroup,
                2,
                2),
        f=50,
        V1=100,
        V2=100,
        SNominal=30E3,
        v_sc=0.05,
        P_sc=300) annotation (Placement(transformation(extent={{-10,
                40},{10,60}})));
      Modelica.Electrical.Machines.BasicMachines.Transformers.Dy.Dy01
        transformer(
        n=transformerData.n,
        R1=transformerData.R1,
        L1sigma=transformerData.L1sigma,
        R2=transformerData.R2,
        L2sigma=transformerData.L2sigma,
        T1Ref=293.15,
        alpha20_1(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
        T2Ref=293.15,
        alpha20_2(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
        T1Operational=293.15,
        T2Operational=293.15) annotation (Placement(transformation(
              extent={{-20,-10},{20,30}})));

      Modelica.Electrical.Polyphase.Basic.PlugToPin_n plugToPin_n(k=1) annotation (Placement(transformation(extent={{20,0},{40,20}})));
      Modelica.Electrical.Analog.Basic.Resistor load(R=RL)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={50,0})));
      Modelica.Electrical.Analog.Basic.Resistor earth(R=1e6)
        annotation (Placement(transformation(
            origin={-10,-40},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.Analog.Basic.Ground groundT annotation (
          Placement(transformation(extent={{-20,-80},{0,-60}})));
    initial equation
      transformer.i2[1] = 0;
    equation
      connect(starS.pin_n, groundS.p)
        annotation (Line(points={{-90,-50},{-90,-60}}, color={0,0,255}));
      connect(source.plug_n, starS.plug_p)
        annotation (Line(points={{-90,-20},{-90,-30}}, color={0,0,255}));
      connect(currentSensorS.plug_n, transformer.plug1) annotation (Line(
          points={{-40,10},{-20,10}},
          color={0,0,255}));
      connect(transformer.plug2, plugToPin_n.plug_n) annotation (Line(
          points={{20,10},{28,10}},
          color={0,0,255}));
      connect(transformer.starpoint2, groundL.p) annotation (Line(
          points={{10,-10},{10,-60}},
          color={0,0,255}));
      connect(load.p, plugToPin_n.pin_n) annotation (Line(
          points={{50,10},{32,10}},
          color={0,0,255}));
      connect(transformer.starpoint2, load.n) annotation (Line(
          points={{10,-10},{50,-10}},
          color={0,0,255}));
      connect(source.plug_p, currentSensorS.plug_p) annotation (Line(
          points={{-90,0},{-90,10},{-60,10}},
          color={0,0,255}));
      connect(earth.n, groundT.p) annotation (Line(points={{-10,-50},{-10,-50},
              {-10,-60}}, color={0,0,255}));
      annotation (Documentation(info="<html>
<h4>Asymmetrical (singlephase) load:</h4>
<p>
You may choose different connections.
</p>
<p>
<b>Please pay attention</b> to proper grounding of the primary and secondary part of the whole circuit.<br>
The primary and secondary starpoint are available as connectors, if the connection is not delta (D or d).
</p>
<p>
In some cases it may be necessary to ground the transformer's starpoint even though the source's or load's starpoint are grounded:
</p>
<ul>
<li>Yy with primary starpoint connected to source's starpoint: primary current in only one phase</li>
<li>Yy primary starpoint  not connected to source's starpoint: secondary voltage breaks down</li>
<li>Yz ... Grounding of transformer's primary starpoint with reasonable high earthing resistance is necessary.</li>
<li>Dy ... Load current in two   primary phases.</li>
<li>Dz ... Load current in three primary phases.</li>
</ul>
</html>"),
         experiment(StopTime=0.1, Interval=0.001));
    end AsymmetricalLoad;
                         annotation (
      Icon(graphics={  Polygon(origin = {10, 10}, fillColor = {135, 135, 135}, fillPattern = FillPattern.VerticalCylinder, points = {{60, 50}, {40, 30}, {40, -50}, {60, -70}, {60, 50}}),                                                                                                                                                                                                        Rectangle(origin = {10, 10}, lineColor = {128, 0, 255}, fillColor = {128, 0, 255},
              fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder, extent = {{32, -46}, {68, 26}}),
                                                                                                                                                                                                        Polygon(origin = {10, 10}, fillColor = {135, 135, 135},
              fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder, points = {{-10, 40}, {-20, 30}, {-20, -50}, {-10, -60}, {0, -50}, {0, 30}, {-10, 40}}),                                                                                                                                                                                                        Rectangle(origin = {10, 8}, lineColor = {128, 0, 255}, fillColor = {128, 0, 255},
              fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder, extent = {{-28, -46}, {8, 26}}),
                                                                                                                                                                                           Polygon(origin = {10, 10}, fillColor = {135, 135, 135}, fillPattern = FillPattern.VerticalCylinder, points = {{-80, 50}, {60, 50}, {40, 30}, {0, 30}, {-10, 40}, {-20, 30}, {-60, 30}, {-80, 50}}), Rectangle(origin = {10, 10}, lineColor = {0, 128, 255}, fillColor = {0, 128, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{26, -38}, {74, 18}}), Polygon(origin = {10, 10}, fillColor = {135, 135, 135},
              fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder, points = {{-80, 50}, {-60, 30}, {-60, -50}, {-80, -70}, {-80, 50}}), Polygon(origin = {10, 10}, fillColor = {135, 135, 135},
              fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder, points = {{-80, -70}, {60, -70}, {40, -50}, {0, -50}, {-10, -60}, {-20, -50}, {-60, -50}, {-80, -70}}), Rectangle(origin = {10, 10}, lineColor = {0, 128, 255}, fillColor = {0, 128, 255},
              fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder, extent = {{-34, -38}, {14, 18}}), Rectangle(origin = {10, 10}, lineColor = {128, 0, 255}, fillColor = {128, 0, 255},
              fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder, extent = {{-88, -46}, {-52, 26}}),                                                                                                                                                                                             Rectangle(origin = {10, 12}, lineColor = {0, 128, 255}, fillColor = {0, 128, 255},
              fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder, extent = {{-94, -38}, {-46, 18}})}));
  end Transformer;

  package ASM
    extends Modelica.Icons.Package;
    model AIMC_DOL_SlipCalculation
      extends AIMC_DOL(fNominal=60, aimc(p=3));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed(displayUnit="rpm") = 52.35987755983) annotation (Placement(transformation(extent={{90,-50},{70,-30}})));
    equation
      connect(loadInertia.flange_b, constantSpeed.flange) annotation (Line(points={{60,-40},{70,-40}}, color={0,0,0}));
    end AIMC_DOL_SlipCalculation;

    model AIMC_DOL
      "Test example 1: AsynchronousInductionMachineSquirrelCage direct-on-line"
      extends Modelica.Icons.Example;
      constant Integer m=3 "number of phases";
      parameter Modelica.Units.SI.Voltage VNominal=100 "nominal RMS voltage per phase";
      parameter Modelica.Units.SI.Frequency fNominal=50 "nominal frequency";
      parameter Modelica.Units.SI.Time tStart1=0.1 "start time";
      parameter Modelica.Units.SI.Torque TLoad=161.4 "nominal load torque";
      parameter Modelica.Units.SI.AngularVelocity wLoad(displayUnit="1/min") = 1440.45*2*Modelica.Constants.pi/60 "nominal load speed";
      parameter Modelica.Units.SI.Inertia JLoad=0.9 "load's moment of inertia";
      Modelica.Electrical.Machines.BasicMachines.InductionMachines.IM_SquirrelCage aimc annotation (Placement(transformation(extent={{-20,-50},{0,-30}}, rotation=0)));
      Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor
        currentQuasiRMSSensor
        annotation (Placement(transformation(
            origin={0,0},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.Polyphase.Sources.SineVoltage sineVoltage(
        final m=m,
        f=fill(fNominal, m),
        V=fill(sqrt(2/3)*VNominal, m)) annotation (Placement(transformation(
            origin={0,60},
            extent={{10,-10},{-10,10}},
            rotation=270)));
      Modelica.Electrical.Polyphase.Basic.Star star(final m=m) annotation (Placement(transformation(extent={{-50,80},{-70,100}}, rotation=0)));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(
            origin={-90,90},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Blocks.Sources.BooleanStep booleanStep[m](each startTime=tStart1)
        annotation (Placement(transformation(extent={{-80,30},{-60,50}},
              rotation=0)));
      Modelica.Electrical.Polyphase.Ideal.IdealClosingSwitch idealCloser(final m=m) annotation (Placement(transformation(
            origin={0,30},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Mechanics.Rotational.Components.Inertia loadInertia(
                                                        J=JLoad)
        annotation (Placement(transformation(extent={{40,-50},{60,-30}},
              rotation=0)));
      Modelica.Electrical.Machines.Utilities.TerminalBox TerminalBox1(
          terminalConnection="D")
        annotation (Placement(transformation(extent={{-20,-34},{0,-14}},
              rotation=0)));
    equation
      connect(star.pin_n, ground.p)
        annotation (Line(points={{-70,90},{-80,90}}, color={0,0,255}));
      connect(sineVoltage.plug_n, star.plug_p)
        annotation (Line(points={{1.33731e-15,70},{1.33731e-15,90},{-50,90}},
            color={0,0,255}));
      connect(sineVoltage.plug_p, idealCloser.plug_p)
        annotation (Line(points={{-2.33651e-15,50},{0,48},{1.22461e-15,46},{
              2.33651e-15,46},{2.33651e-15,40}},   color={0,0,255}));
      connect(booleanStep.y, idealCloser.control)   annotation (Line(points={{
              -59,40},{-20,40},{-20,30},{-7,30}}, color={255,0,255}));
      connect(idealCloser.plug_n, currentQuasiRMSSensor.plug_p)
                                                             annotation (Line(
          points={{-1.33731e-15,20},{-1.33731e-15,17},{2.33651e-15,17},{
              2.33651e-15,10}},
          color={0,0,255}));
      connect(TerminalBox1.plug_sn, aimc.plug_sn)  annotation (Line(
          points={{-16,-30},{-16,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(TerminalBox1.plug_sp, aimc.plug_sp)  annotation (Line(
          points={{-4,-30},{-4,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(TerminalBox1.plugSupply, currentQuasiRMSSensor.plug_n)
                                                                 annotation (Line(
          points={{-10,-28},{-10,-20},{-1.33731e-15,-20},{-1.33731e-15,-10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(aimc.flange, loadInertia.flange_a) annotation (Line(
          points={{5.55112e-16,-40},{40,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}),
                graphics),
        experiment(StopTime=1.5, Interval=0.001),
        Documentation(info="<HTML>
<b>1st Test example: Asynchronous induction machine with squirrel cage - direct on line starting</b><br>
At start time tStart three phase voltage is supplied to the asynchronous induction machine with squirrel cage;
the machine starts from standstill, accelerating inertias against load torque quadratic dependent on speed, finally reaching nominal speed.<br>
Simulate for 1.5 seconds and plot (versus time):
<ul>
<li>CurrentRMSSensor1.I: stator current RMS</li>
<li>AIMC1.rpmMechanical: motor's speed</li>
<li>AIMC1.tauElectrical: motor's torque</li>
</ul>
Default machine parameters of model <i>AIM_SquirrelCage</i> are used.
</HTML>"));
    end AIMC_DOL;

    model AIMC_TorqueChar "Torque characteristic of a ASM"
      extends Modelica.Icons.Example;
      constant Integer m=3 "number of phases";
      parameter Modelica.Units.SI.Voltage VNominal=100 "nominal RMS voltage per phase";
      parameter Modelica.Units.SI.Frequency fNominal=50 "nominal frequency";
      parameter Modelica.Units.SI.Time tStart1=0.1 "start time";
      parameter Modelica.Units.SI.Torque TLoad=161.4 "nominal load torque";
      parameter Modelica.Units.SI.AngularVelocity wLoad(displayUnit="1/min") = 1440.45*2*Modelica.Constants.pi/60 "nominal load speed";
      parameter Modelica.Units.SI.Inertia JLoad=0.9 "load's moment of inertia";
      Modelica.Electrical.Machines.BasicMachines.InductionMachines.IM_SquirrelCage aimc annotation (Placement(transformation(extent={{-20,-50},{0,-30}}, rotation=0)));
      Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor
        currentQuasiRMSSensor
        annotation (Placement(transformation(
            origin={0,0},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.Polyphase.Sources.SineVoltage sineVoltage(
        final m=m,
        f=fill(fNominal, m),
        V=fill(sqrt(2/3)*VNominal, m)) annotation (Placement(transformation(
            origin={0,60},
            extent={{10,-10},{-10,10}},
            rotation=270)));
      Modelica.Electrical.Polyphase.Basic.Star star(final m=m) annotation (Placement(transformation(extent={{-50,80},{-70,100}}, rotation=0)));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(
            origin={-90,90},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Blocks.Sources.BooleanStep booleanStep[m](each startTime=tStart1)
        annotation (Placement(transformation(extent={{-80,30},{-60,50}},
              rotation=0)));
      Modelica.Electrical.Polyphase.Ideal.IdealClosingSwitch idealCloser(final m=m) annotation (Placement(transformation(
            origin={0,30},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Mechanics.Rotational.Components.Inertia loadInertia(
                                                        J=JLoad)
        annotation (Placement(transformation(extent={{10,-50},{30,-30}},
              rotation=0)));
      Modelica.Mechanics.Rotational.Sources.Speed LoadTorque(useSupport=false)
        annotation (Placement(transformation(extent={{60,-50},{40,-30}},
              rotation=0)));
      Modelica.Electrical.Machines.Utilities.TerminalBox TerminalBox1(
          terminalConnection="D")
        annotation (Placement(transformation(extent={{-20,-34},{0,-14}},
              rotation=0)));
      Modelica.Blocks.Sources.Ramp ramp(
        height=3000,
        duration=2,
        offset=0,
        startTime=tStart1)
                  annotation (Placement(transformation(extent={{40,-80},{60,-60}})));
      Modelica.Blocks.Math.UnitConversions.From_rpm from_rpm
        annotation (Placement(transformation(extent={{72,-80},{92,-60}})));
    equation
      connect(star.pin_n, ground.p)
        annotation (Line(points={{-70,90},{-80,90}}, color={0,0,255}));
      connect(sineVoltage.plug_n, star.plug_p)
        annotation (Line(points={{1.33731e-15,70},{1.33731e-15,90},{-50,90}},
            color={0,0,255}));
      connect(sineVoltage.plug_p, idealCloser.plug_p)
        annotation (Line(points={{-2.33651e-15,50},{0,48},{1.22461e-15,46},{
              2.33651e-15,46},{2.33651e-15,40}},   color={0,0,255}));
      connect(booleanStep.y, idealCloser.control)   annotation (Line(points={{-59,40},{-20,40},{-20,30},{-12,30}},
                                                  color={255,0,255}));
      connect(idealCloser.plug_n, currentQuasiRMSSensor.plug_p)
                                                             annotation (Line(
          points={{-1.33731e-15,20},{-1.33731e-15,17},{2.33651e-15,17},{
              2.33651e-15,10}},
          color={0,0,255}));
      connect(TerminalBox1.plug_sn, aimc.plug_sn)  annotation (Line(
          points={{-16,-30},{-16,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(TerminalBox1.plug_sp, aimc.plug_sp)  annotation (Line(
          points={{-4,-30},{-4,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(TerminalBox1.plugSupply, currentQuasiRMSSensor.plug_n)
                                                                 annotation (Line(
          points={{-10,-28},{-10,-20},{-1.33731e-15,-20},{-1.33731e-15,-10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(loadInertia.flange_b, LoadTorque.flange)          annotation (
          Line(
          points={{30,-40},{40,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(aimc.flange, loadInertia.flange_a) annotation (Line(
          points={{5.55112e-16,-40},{10,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(ramp.y, from_rpm.u) annotation (Line(
          points={{61,-70},{70,-70}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(from_rpm.y, LoadTorque.w_ref) annotation (Line(
          points={{93,-70},{100,-70},{100,-40},{62,-40}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}),
                graphics),
        experiment(StopTime=2, Interval=0.001),
        Documentation(info="<HTML>
<b>1st Test example: Asynchronous induction machine with squirrel cage - direct on line starting</b><br>
At start time tStart three phase voltage is supplied to the asynchronous induction machine with squirrel cage;
the machine starts from standstill, accelerating inertias against load torque quadratic dependent on speed, finally reaching nominal speed.<br>
Simulate for 1.5 seconds and plot (versus time):
<ul>
<li>CurrentRMSSensor1.I: stator current RMS</li>
<li>AIMC1.rpmMechanical: motor's speed</li>
<li>AIMC1.tauElectrical: motor's torque</li>
</ul>
Default machine parameters of model <i>AIM_SquirrelCage</i> are used.
</HTML>"));
    end AIMC_TorqueChar;

    model ASM_SlipRingStart
      extends Modelica.Electrical.Machines.Examples.InductionMachines.IMS_Start;
    end ASM_SlipRingStart;

    model ASM_DirectStart
      extends Modelica.Electrical.Machines.Examples.InductionMachines.IMC_YD(booleanStepYD.startTime = {0, 0, 0});
      annotation (
      //experiment(StopTime=20, Interval=1E-4),
       Commands(file = "plotResults.mos" "plotResults"),
        experiment(StartTime = 0, StopTime = 2.5, Tolerance = 1e-6, Interval = 0.0001));
    end ASM_DirectStart;

    model ASM_YDStart
  extends Modelica.Electrical.Machines.Examples.InductionMachines.IMC_YD;
      annotation(
        experiment(StartTime = 0, StopTime = 2.5, Tolerance = 1e-6, Interval = 0.0001));
    end ASM_YDStart;

    model AIMC_Generator "Torque characteristic of a ASM"
      extends Modelica.Icons.Example;
      constant Integer m=3 "number of phases";
      parameter Modelica.Units.SI.Voltage VNominal=100 "nominal RMS voltage per phase";
      parameter Modelica.Units.SI.Frequency fNominal=50 "nominal frequency";
      parameter Modelica.Units.SI.Time tStart1=0.5 "start time";
      parameter Modelica.Units.SI.Torque TLoad=161.4 "nominal load torque";
      parameter Modelica.Units.SI.AngularVelocity wLoad(displayUnit="1/min") = 1440.45*2*Modelica.Constants.pi/60 "nominal load speed";
      parameter Modelica.Units.SI.Inertia JLoad=0.9 "load's moment of inertia";
      Modelica.Electrical.Machines.BasicMachines.InductionMachines.IM_SquirrelCage aimc annotation (Placement(transformation(extent={{-20,-50},{0,-30}}, rotation=0)));
      Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor
        currentQuasiRMSSensor
        annotation (Placement(transformation(
            origin={0,20},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.Polyphase.Sources.SineVoltage sineVoltage(
        final m=m,
        f=fill(fNominal, m),
        V=fill(sqrt(2/3)*VNominal, m)) annotation (Placement(transformation(
            origin={0,80},
            extent={{10,-10},{-10,10}},
            rotation=270)));
      Modelica.Electrical.Polyphase.Basic.Star star(final m=m) annotation (Placement(transformation(extent={{-50,80},{-70,100}}, rotation=0)));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(
            origin={-90,90},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Blocks.Sources.BooleanStep booleanStep[m](each startTime=tStart1)
        annotation (Placement(transformation(extent={{-80,40},{-60,60}},
              rotation=0)));
      Modelica.Electrical.Polyphase.Ideal.IdealClosingSwitch idealCloser(final m=m) annotation (Placement(transformation(
            origin={0,50},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Mechanics.Rotational.Components.Inertia loadInertia(
                                                        J=JLoad)
        annotation (Placement(transformation(extent={{10,-50},{30,-30}},
              rotation=0)));
      Modelica.Mechanics.Rotational.Sources.Speed LoadTorque(useSupport=false)
        annotation (Placement(transformation(extent={{60,-50},{40,-30}},
              rotation=0)));
      Modelica.Electrical.Machines.Utilities.TerminalBox TerminalBox1(
          terminalConnection="D")
        annotation (Placement(transformation(extent={{-20,-34},{0,-14}},
              rotation=0)));
      Modelica.Blocks.Sources.Ramp ramp(
        height=200,
        duration=2,
        offset=1400,
        startTime=tStart1)
                  annotation (Placement(transformation(extent={{40,-80},{60,-60}})));
      Modelica.Blocks.Math.UnitConversions.From_rpm from_rpm
        annotation (Placement(transformation(extent={{72,-80},{92,-60}})));
      Modelica.Electrical.Machines.Sensors.ElectricalPowerSensor powerSensor annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-8})));
    equation
      connect(star.pin_n, ground.p)
        annotation (Line(points={{-70,90},{-80,90}}, color={0,0,255}));
      connect(sineVoltage.plug_n, star.plug_p)
        annotation (Line(points={{1.77636e-15,90},{-50,90}},
            color={0,0,255}));
      connect(sineVoltage.plug_p, idealCloser.plug_p)
        annotation (Line(points={{-1.77636e-15,70},{-2.33651e-15,70},{-2.33651e-15,46},{2.33651e-15,46},{2.33651e-15,60}},
                                                   color={0,0,255}));
      connect(booleanStep.y, idealCloser.control)   annotation (Line(points={{-59,50},{-12,50}},
                                                  color={255,0,255}));
      connect(idealCloser.plug_n, currentQuasiRMSSensor.plug_p)
                                                             annotation (Line(
          points={{-1.33731e-15,40},{-1.33731e-15,17},{2.33651e-15,17},{2.33651e-15,30}},
          color={0,0,255}));
      connect(TerminalBox1.plug_sn, aimc.plug_sn)  annotation (Line(
          points={{-16,-30},{-16,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(TerminalBox1.plug_sp, aimc.plug_sp)  annotation (Line(
          points={{-4,-30},{-4,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(loadInertia.flange_b, LoadTorque.flange)          annotation (
          Line(
          points={{30,-40},{40,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(aimc.flange, loadInertia.flange_a) annotation (Line(
          points={{5.55112e-16,-40},{10,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(ramp.y, from_rpm.u) annotation (Line(
          points={{61,-70},{70,-70}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(from_rpm.y, LoadTorque.w_ref) annotation (Line(
          points={{93,-70},{100,-70},{100,-40},{62,-40}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(TerminalBox1.plugSupply, powerSensor.plug_ni) annotation (Line(points={{-10,-28},{-10,-20},{-1.33731e-15,-20},{-1.77636e-15,-18}}, color={0,0,255}));
      connect(powerSensor.plug_p, currentQuasiRMSSensor.plug_n) annotation (Line(points={{1.77636e-15,2},{1.77636e-15,6},{-1.33731e-15,6},{-1.33731e-15,10}}, color={0,0,255}));
      connect(powerSensor.plug_nv, TerminalBox1.plug_sn) annotation (Line(points={{-10,-8},{-16,-8},{-16,-30}}, color={0,0,255}));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}})),
        experiment(StopTime=3, Interval=0.001, StartTime = 0, Tolerance = 1e-06),
        Documentation(info="<HTML>
<b>1st Test example: Asynchronous induction machine with squirrel cage - direct on line starting</b><br>
At start time tStart three phase voltage is supplied to the asynchronous induction machine with squirrel cage;
the machine starts from standstill, accelerating inertias against load torque quadratic dependent on speed, finally reaching nominal speed.<br>
Simulate for 1.5 seconds and plot (versus time):
<ul>
<li>CurrentRMSSensor1.I: stator current RMS</li>
<li>AIMC1.rpmMechanical: motor's speed</li>
<li>AIMC1.tauElectrical: motor's torque</li>
</ul>
Default machine parameters of model <i>AIM_SquirrelCage</i> are used.
</HTML>"));
    end AIMC_Generator;
                       annotation (
      Icon(graphics={  Rectangle(origin = {2.835, 10}, fillColor = {0, 128, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-60, -60}, {60, 60}}), Rectangle(origin = {2.835, 10}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid, extent = {{-60, 50}, {20, 70}}), Polygon(origin = {2.835, 10}, fillPattern = FillPattern.Solid, points = {{-70, -90}, {-60, -90}, {-30, -20}, {20, -20}, {50, -90}, {60, -90}, {60, -100}, {-70, -100}, {-70, -90}}), Rectangle(origin = {2.835, 10}, fillColor = {95, 95, 95},
              fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, extent = {{60, -10}, {80, 10}}), Rectangle(origin = {2.835, 10}, fillColor = {128, 128, 128},
              fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, extent = {{-80, -60}, {-60, 60}})}));
  end ASM;

  package SM
    extends Modelica.Icons.Package;
    model EqvSM
    extends Modelica.Icons.Example;
      Modelica.Electrical.Analog.Sources.SineVoltage E0(f=50, V=230) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-40,30})));
      Modelica.Electrical.Analog.Sources.SineVoltage E(f=50, V=230) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={60,30})));
      Modelica.Electrical.Analog.Basic.Inductor Xs
        annotation (Placement(transformation(extent={{-20,50},{0,70}})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-50,-36},{-30,-16}})));
      Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor
        annotation (Placement(transformation(extent={{20,50},{40,70}})));
    equation
      connect(E0.p, Xs.p) annotation (Line(
          points={{-40,40},{-40,60},{-20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(E0.n, E.n) annotation (Line(
          points={{-40,20},{-40,0},{60,0},{60,20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ground.p, E0.n) annotation (Line(
          points={{-40,-16},{-40,20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(Xs.n, powerSensor.pc)
        annotation (Line(points={{5.55112e-16,60},{20,60}}, color={0,0,255}));
      connect(powerSensor.nc, E.p)
        annotation (Line(points={{40,60},{60,60},{60,40}}, color={0,0,255}));
      connect(powerSensor.pv, powerSensor.pc) annotation (Line(points={{30,70},
              {30,80},{10,80},{10,60},{20,60}}, color={0,0,255}));
      connect(powerSensor.nv, E.n) annotation (Line(points={{30,50},{30,0},{60,
              0},{60,20}}, color={0,0,255}));
      annotation (
        experiment(StopTime=0.04),
        __Dymola_experimentSetupOutput);
    end EqvSM;

    model SMEE_Generator
      extends Modelica.Electrical.Machines.Examples.SynchronousMachines.SMEE_Generator;
    end SMEE_Generator;

    model SMEE_ShortCircuit
      "Test example: ElectricalExcitedSynchronousInductionMachine with voltage controller"
      extends Modelica.Icons.Example;
      import Modelica.Constants.pi;
      constant Integer m=3 "Number of phases";
      parameter Modelica.Units.SI.AngularVelocity wNominal=2*pi*smeeData.fsNominal/smee.p "Nominal speed";
      parameter Modelica.Units.SI.Impedance ZNominal=3*smeeData.VsNominal^2/smeeData.SNominal "Nominal load impedance";
      parameter Real powerFactor(
        min=0,
        max=1) = 0.8 "Load power factor";
      parameter Modelica.Units.SI.Resistance RLoad=ZNominal*powerFactor "Load resistance";
      parameter Modelica.Units.SI.Inductance LLoad=ZNominal*sqrt(1 - powerFactor^2)/(2*pi*smeeData.fsNominal) "Load inductance";
      parameter Modelica.Units.SI.Voltage Ve0=smee.IeOpenCircuit*Modelica.Electrical.Machines.Thermal.convertResistance(
              smee.Re,
              smee.TeRef,
              smee.alpha20e,
              smee.TeOperational) "No load excitation voltage";
      parameter Real k=2*Ve0/smeeData.VsNominal "Voltage controller: gain";
      parameter Modelica.Units.SI.Time Ti=smeeData.Td0Transient/2 "Voltage controller: integral time constant";
      output Real controlError=(setPointGain.y - voltageQuasiRMSSensor.V)/
          smeeData.VsNominal;
      Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_ElectricalExcited smee(
        fsNominal=smeeData.fsNominal,
        Rs=smeeData.Rs,
        TsRef=smeeData.TsRef,
        Lssigma=smeeData.Lssigma,
        Lmd=smeeData.Lmd,
        Lmq=smeeData.Lmq,
        Lrsigmad=smeeData.Lrsigmad,
        Lrsigmaq=smeeData.Lrsigmaq,
        Rrd=smeeData.Rrd,
        Rrq=smeeData.Rrq,
        TrRef=smeeData.TrRef,
        VsNominal=smeeData.VsNominal,
        IeOpenCircuit=smeeData.IeOpenCircuit,
        Re=smeeData.Re,
        TeRef=smeeData.TeRef,
        sigmae=smeeData.sigmae,
        useDamperCage=true,
        p=2,
        Jr=0.29,
        Js=0.29,
        statorCoreParameters(VRef=100),
        strayLoadParameters(IRef=100),
        brushParameters(ILinear=0.01),
        TsOperational=293.15,
        alpha20s=smeeData.alpha20s,
        TrOperational=293.15,
        alpha20r=smeeData.alpha20r,
        alpha20e=smeeData.alpha20e,
        TeOperational=293.15) annotation (Placement(transformation(extent={{0,-40},{20,-20}}, rotation=0)));
      parameter Modelica.Electrical.Machines.Utilities.SynchronousMachineData smeeData(
        SNominal=30e3,
        VsNominal=100,
        fsNominal=50,
        IeOpenCircuit=10,
        x0=0.1,
        xd=1.6,
        xq=1.6,
        xdTransient=0.1375,
        xdSubtransient=0.121428571,
        xqSubtransient=0.148387097,
        Ta=0.014171268,
        Td0Transient=0.261177343,
        Td0Subtransient=0.006963029,
        Tq0Subtransient=0.123345081,
        TsSpecification=293.15,
        TsRef=293.15,
        alpha20s(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
        TrSpecification=293.15,
        TrRef=293.15,
        alpha20r(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
        TeSpecification=293.15,
        TeRef=293.15,
        alpha20e(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero)
        annotation (Placement(transformation(extent={{0,-70},{20,-50}})));

      Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(
          terminalConnection="Y") annotation (Placement(transformation(extent={
                {0,-20},{20,0}}, rotation=0)));
      Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
            transformation(
            origin={-90,0},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      Modelica.Mechanics.Rotational.Sources.Speed speed
        annotation (Placement(transformation(extent={{50,-40},{30,-20}})));
      Modelica.Blocks.Sources.Ramp speedRamp(height=wNominal, duration=1)
        annotation (Placement(transformation(extent={{80,-40},{60,-20}})));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={30,-50})));
      Modelica.Blocks.Math.Gain setPointGain(k=(smeeData.VsNominal/wNominal)/
            unitMagneticFlux)
        annotation (Placement(transformation(extent={{-50,-90},{-70,-70}})));
      Modelica.Electrical.Machines.Sensors.VoltageQuasiRMSSensor voltageQuasiRMSSensor(
          ToSpacePhasor1(y(each start=1E-3, each fixed=true))) annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,0})));
      Modelica.Blocks.Continuous.LimPID voltageController(
        controllerType=Modelica.Blocks.Types.SimpleController.PI,
        k=k,
        Ti=Ti,
        yMax=2.5*Ve0,
        yMin=0,
        initType=Modelica.Blocks.Types.Init.InitialState,
        Td=0.001) annotation (Placement(transformation(extent={{-70,-20},{-50,-40}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage excitationVoltage
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-30,-30})));
      Modelica.Electrical.Analog.Basic.Ground groundExcitation annotation (
          Placement(transformation(
            origin={-30,-60},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensor
        annotation (Placement(transformation(
            origin={10,30},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Blocks.Sources.BooleanPulse loadControl(          startTime=2, period=
            10)
        annotation (Placement(transformation(extent={{-40,70},{-20,90}})));
      Modelica.Electrical.Polyphase.Ideal.CloserWithArc switch(
        m=m,
        Ron=fill(1e-5, m),
        Goff=fill(1e-5, m),
        V0=fill(30, m),
        dVdt=fill(10e3, m),
        Vmax=fill(60, m)) annotation (Placement(transformation(extent={{30,40},{50,60}})));
      Modelica.Electrical.Polyphase.Basic.Resistor loadResistor(m=m, R=fill(RLoad, m)) annotation (Placement(transformation(extent={{-20,40},{-40,60}})));
      Modelica.Electrical.Polyphase.Basic.Inductor loadInductor(m=m, L=fill(LLoad, m)) annotation (Placement(transformation(extent={{-60,40},{-80,60}})));
      Modelica.Electrical.Polyphase.Basic.Star star(m=m) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-90,30})));
    protected
      constant Modelica.Units.SI.MagneticFlux unitMagneticFlux=1 annotation (HideResult=true);
    public
      Modelica.Electrical.Analog.Basic.Ground ground1
                                                     annotation (Placement(
            transformation(
            origin={60,0},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      Modelica.Electrical.Polyphase.Basic.Star star1(m=m) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={60,30})));
    initial equation
      smee.idq_sr = zeros(2);
      //conditional damper cage currents are defined as fixed start values
      smee.ie = 0;

    equation
      connect(terminalBox.plug_sn, smee.plug_sn) annotation (Line(
          points={{4,-16},{4,-20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(terminalBox.plug_sp, smee.plug_sp) annotation (Line(
          points={{16,-16},{16,-20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(excitationVoltage.p, smee.pin_ep) annotation (Line(
          points={{-30,-20},{-20,-20},{-20,-24},{0,-24}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(excitationVoltage.n, smee.pin_en) annotation (Line(
          points={{-30,-40},{-20,-40},{-20,-36},{0,-36}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(excitationVoltage.n, groundExcitation.p) annotation (Line(
          points={{-30,-40},{-30,-50}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageQuasiRMSSensor.plug_n, smee.plug_sn) annotation (Line(
          points={{0,-10},{4,-10},{4,-20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageQuasiRMSSensor.plug_p, smee.plug_sp) annotation (Line(
          points={{0,10},{16,10},{16,-20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(terminalBox.plugSupply, currentQuasiRMSSensor.plug_n)
        annotation (Line(
          points={{10,-14},{10,20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(smee.flange, speed.flange) annotation (Line(
          points={{20,-30},{30,-30}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(speed.flange, speedSensor.flange) annotation (Line(
          points={{30,-30},{30,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(speedRamp.y, speed.w_ref) annotation (Line(
          points={{59,-30},{52,-30}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(setPointGain.y, voltageController.u_s) annotation (Line(
          points={{-71,-80},{-80,-80},{-80,-30},{-72,-30}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(speedSensor.w, setPointGain.u) annotation (Line(
          points={{30,-61},{30,-80},{-48,-80}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(voltageQuasiRMSSensor.V, voltageController.u_m) annotation (
          Line(
          points={{-11,0},{-60,0},{-60,-18}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(voltageController.y, excitationVoltage.v) annotation (Line(
          points={{-49,-30},{-37,-30}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(loadInductor.plug_p, loadResistor.plug_n) annotation (Line(
          points={{-60,50},{-40,50}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(switch.plug_p, currentQuasiRMSSensor.plug_p) annotation (Line(
          points={{30,50},{10,50},{10,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star.plug_p, loadInductor.plug_n) annotation (Line(
          points={{-90,40},{-90,50},{-80,50}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(loadControl.y, switch.control[1]) annotation (Line(
          points={{-19,80},{40,80},{40,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(loadControl.y, switch.control[2]) annotation (Line(
          points={{-19,80},{40,80},{40,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(loadControl.y, switch.control[3]) annotation (Line(
          points={{-19,80},{40,80},{40,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(star.pin_n, ground.p) annotation (Line(
          points={{-90,20},{-90,10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star1.pin_n, ground1.p) annotation (Line(
          points={{60,20},{60,10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(switch.plug_n, star1.plug_p)
        annotation (Line(points={{50,50},{60,50},{60,40}}, color={0,0,255}));
      connect(loadResistor.plug_p, currentQuasiRMSSensor.plug_p)
        annotation (Line(points={{-20,50},{10,50},{10,40}}, color={0,0,255}));
      annotation (experiment(StopTime=10, Interval=0.001), Documentation(info=
             "<html>
<b>Test example: Electrical excited synchronous induction machine with voltage controller</b><br>
An electrically excited synchronous generator is started with a speed ramp, then driven with constant speed.
Voltage is controlled, the set point depends on speed. After start-up the generator is loaded, the load is rejected.
Simulate for 10 seconds and plot:
<ul>
<li>voltageQuasiRMSSensor.V</li>
<li>smee.tauElectrical</li>
<li>smee.ie</li>
</ul>
Default machine parameters of model <i>SM_ElectricalExcited</i> are used.
One could try to optimize the controller parameters.
</html>"));
    end SMEE_ShortCircuit;

    model SMEE_DOL
      extends Modelica.Electrical.Machines.Examples.SynchronousMachines.SMEE_DOL(torqueStep(offsetTorque=1));
    end SMEE_DOL;
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100.0,-100.0},{100.0,100.0}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100.0,-100.0},{100.0,100.0}},
            radius=25.0),
                       Rectangle(origin = {2.835, 10}, fillColor={162,29,33},     fillPattern=FillPattern.HorizontalCylinder,   extent = {{-60, -60}, {60, 60}},
            lineColor={0,0,0}),                                                                                                                                   Rectangle(origin = {2.835, 10}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid, extent = {{-60, 50}, {20, 70}}), Polygon(origin = {2.835, 10}, fillPattern = FillPattern.Solid, points = {{-70, -90}, {-60, -90}, {-30, -20}, {20, -20}, {50, -90}, {60, -90}, {60, -100}, {-70, -100}, {-70, -90}}), Rectangle(origin = {2.835, 10}, fillColor = {95, 95, 95},
              fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, extent = {{60, -10}, {80, 10}}), Rectangle(origin = {2.835, 10}, fillColor = {128, 128, 128},
              fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, extent = {{-80, -60}, {-60, 60}})}));
  end SM;

  package SinglePhase
    extends Modelica.Icons.Package;
    model RevolvingMMFs
      extends Modelica.Icons.Example;
      Modelica.Blocks.Sources.Sine sine1(
        amplitude=85,
        f=0.5,
        phase=0) annotation (Placement(transformation(extent={{-60,0},{-40,20}})));
      Modelica.Blocks.Sources.Sine sine2(
        amplitude=85,
        f=0.5,
        phase=0) annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
      Modelica.Blocks.Sources.Ramp ramp(
        duration=1,
        height=3.14,
        offset=-3.14/2)
        annotation (Placement(transformation(extent={{68,-100},{80,-88}})));
      Modelica.Blocks.Math.Add add
        annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
      Modelica.Blocks.Math.UnitConversions.To_deg to_deg
        annotation (Placement(transformation(extent={{88,-100},{100,-88}})));
      Modelica.Blocks.Sources.Sine sine(
        f=0.5,
        phase=0,
        amplitude=170) annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
      Modelica.Blocks.Math.Gain gain
        annotation (Placement(transformation(extent={{-28,66},{-20,74}})));
    equation
      connect(sine1.y, add.u1) annotation (Line(
          points={{-39,10},{-30,10},{-30,-4},{-22,-4}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(sine2.y, add.u2) annotation (Line(
          points={{-39,-30},{-30,-30},{-30,-16},{-22,-16}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(ramp.y, to_deg.u) annotation (Line(
          points={{80.6,-94},{86.8,-94}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(sine.y, gain.u) annotation (Line(
          points={{-39,70},{-28.8,70}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics));
    end RevolvingMMFs;

    model AIMC_Steinmetz
      extends Modelica.Electrical.Machines.Examples.InductionMachines.IMC_Steinmetz;
    end AIMC_Steinmetz;
    annotation (Icon(graphics={
                             Line(
          origin={10,20},
          points={{-90,-20},{-78.7,14.2},{-71.5,33.1},{-65.1,46.4},{-59.4,54.6},{
              -53.8,59.1},{-48.2,59.8},{-42.6,56.6},{-36.9,49.7},{-31.3,39.4},{-24.9,
              24.1},{-16.83,1.2},{0.1,-50.8},{7.3,-70.2},{13.7,-84.2},{19.3,-93.1},
              {25,-98.4},{30.6,-100},{36.2,-97.6},{41.9,-91.5},{47.5,-81.9},{53.9,
              -67.2},{62,-44.8},{70,-20}},
          smooth=Smooth.Bezier)}));
  end SinglePhase;

  package DC
    extends Modelica.Icons.Package;
    model DCEE_Start
      "Test example: DC with electrical ecxitation starting with voltage ramp"
      extends Modelica.Icons.Example;
      parameter Modelica.Units.SI.Voltage Va=100 "Actual armature voltage";
      parameter Modelica.Units.SI.Time tStart=0.2 "Start of armature voltage ramp";
      parameter Modelica.Units.SI.Time tRamp=0.8 "Armature voltage ramp";
      parameter Modelica.Units.SI.Voltage Ve=100 "Actual excitation voltage";
      parameter Modelica.Units.SI.Torque TLoad=63.66 "Nominal load torque";
      parameter Modelica.Units.SI.Time tStep=1.5 "Time of load torque step";
      parameter Modelica.Units.SI.Inertia JLoad=0.15 "Load's moment of inertia";

      Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_ElectricalExcited dcee
        annotation (Placement(transformation(extent={{-20,-50},{0,-30}},
              rotation=0)));
      Modelica.Blocks.Sources.Ramp armature(
        duration=tRamp,
        height=Va,
        startTime=tStart)
        annotation (Placement(transformation(extent={{-80,60},{-60,80}},
              rotation=0)));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(extent={{0,30},{-20,50}}, rotation=
               0)));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(
            origin={-70,40},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Electrical.Analog.Sources.SignalVoltage   fieldExcitation
        annotation (Placement(transformation(
            origin={-40,-40},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.Analog.Basic.Ground groundExcitation
        annotation (Placement(transformation(
            origin={-40,-90},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      Modelica.Mechanics.Rotational.Components.Inertia loadInertia(
                                                        J=JLoad)
        annotation (Placement(transformation(extent={{40,-50},{60,-30}},
              rotation=0)));
      Modelica.Mechanics.Rotational.Sources.TorqueStep loadTorqueStep(
                                                              startTime=tStep,
          stepTorque=-TLoad,
        useSupport=false)
                    annotation (Placement(transformation(extent={{90,-50},{70,
                -30}}, rotation=0)));
      Modelica.Blocks.Sources.Ramp field(
        duration=tRamp,
        height=-Ve,
        offset=Ve,
        startTime=2)
        annotation (Placement(transformation(extent={{-80,-50},{-60,-30}},
              rotation=0)));
    equation
      connect(armature.y, signalVoltage.v)
                                         annotation (Line(points={{-59,70},{-10,70},{-10,52}},
                             color={0,0,255}));
      connect(signalVoltage.p, dcee.pin_ap)   annotation (Line(points={{
              5.55112e-16,40},{5.55112e-16,-20},{-4,-20},{-4,-30}},
                                         color={0,0,255}));
      connect(signalVoltage.n, ground.p)   annotation (Line(points={{-20,40},{
              -60,40}}, color={0,0,255}));
      connect(dcee.pin_an, ground.p)   annotation (Line(points={{-16,-30},{-16,
              -20},{-20,-20},{-20,40},{-60,40}}, color={0,0,255}));
      connect(fieldExcitation.n, groundExcitation.p)
                                             annotation (Line(points={{-40,-50},
              {-40,-50},{-40,-80}},
                          color={0,0,255}));
      connect(dcee.pin_ep,fieldExcitation. p)   annotation (Line(points={{-20,
              -34},{-30,-34},{-30,-30},{-40,-30}}, color={0,0,255}));
      connect(dcee.pin_en,fieldExcitation. n)   annotation (Line(points={{-20,
              -46},{-30,-46},{-30,-50},{-40,-50}}, color={0,0,255}));
      connect(loadInertia.flange_b, loadTorqueStep.flange)
        annotation (Line(points={{60,-40},{70,-40}}, color={0,0,0}));
      connect(dcee.flange, loadInertia.flange_a) annotation (Line(
          points={{5.55112e-16,-40},{40,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(field.y, fieldExcitation.v) annotation (Line(
          points={{-59,-40},{-52,-40}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},
                {100,100}}),
                graphics),
        experiment(StopTime=2, Interval=0.001),
        Documentation(info="<html>
<h4>Test example: Electrically separate excited DC machine started with an armature voltage ramp</h4>
<p>A voltage ramp is applied to the armature, causing the DC machine to start, and accelerating inertias.</p>
<p>At time tStep a load step is applied.</p>
<p>Simulate for 2 seconds and plot (versus time): </p>
<ul>
<li>dcee.ia: armature current</li>
<li>dcee.wMechanical: motor&apos;s speed</li>
<li>dcee.tauElectrical: motor&apos;s torque</li>
<li>dcee.ie: excitation current</li>
</ul>
<h5>Effect of loosing the excitation:</h5>
<p>Now simulate the model for 3 seconds in total and look what happens to current, torque and speed of the motor.</p>
<p>Default machine parameters of model <i>DC_ElectricalExcited</i> are used. </p>
</html>"),
        __Dymola_experimentSetupOutput);
    end DCEE_Start;
    annotation (Icon(graphics={Line(points={{-60,32},{60,32}}, color={0,0,0}),
                               Line(points={{-60,-30},{60,-30}},
                                                               color={0,0,0})}));
  end DC;
annotation (uses(Modelica(version="4.0.0")), Icon(graphics={Text(
          extent={{-80,80},{80,-80}},
          lineColor={0,0,0},
          textString="USN")}));
end TSE3080Models;