/*
 * Copyright (C) 2014 Pablo Campillo-Sanchez <pabcampi@ucm.es>
 *
 * This software has been developed as part of the 
 * SociAAL project directed by Jorge J. Gomez Sanz
 * (http://grasia.fdi.ucm.es/sociaal)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package server;

import phat.sensors.accelerometer.AccelerationData;
import phat.sensors.Sensor;
import phat.sensors.SensorData;
import phat.sensors.SensorListener;
import java.awt.Color;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RefineryUtilities;

/**
 * An accelerometer listener that plots the x,y,z accelerations of the
 * sensor that is listening.
 * 
 * @author Pablo
 */
public class XYShiftingAccelerationsChart extends ApplicationFrame implements SensorListener {

    XYSeries xAcceleration;
    XYSeries yAcceleration;
    XYSeries zAcceleration;
    
    XYSeriesCollection dataset;    
    JFreeChart chart;
    XYPlot plot;
    int datasize=500;
    
    ChartPanel chartPanel;
    
    float acumulativeTime = 0f;
    
    /**
     * Creates a new graphic to plot accelerations.
     *
     * @param title  the frame title.
     */
    public XYShiftingAccelerationsChart(final String windowstitle, final String chartTitle,
            final String domainAxisLabel, final String rangeAxisLabel) {

        super(windowstitle);

        //Object[][][] data = new Object[3][50][2];
        xAcceleration = new XYSeries("x acc.");        
        yAcceleration = new XYSeries("y acc.");
        zAcceleration = new XYSeries("z acc.");

        dataset = new XYSeriesCollection();
        dataset.addSeries(xAcceleration);
        dataset.addSeries(yAcceleration);
        dataset.addSeries(zAcceleration);

        chart = ChartFactory.createXYLineChart(
                chartTitle, // chart title
                domainAxisLabel, // domain axis label
                rangeAxisLabel, // range axis label
                dataset, // data
                PlotOrientation.VERTICAL,
                true, // include legend
                true,
                false);

        plot = chart.getXYPlot();
        /*final NumberAxis domainAxis = new NumberAxis("x");
        final NumberAxis rangeAxis = new LogarithmicAxis("Log(y)");
        plot.setDomainAxis(domainAxis);
        plot.setRangeAxis(rangeAxis);*/
        chart.setBackgroundPaint(Color.white);
        plot.setOutlinePaint(Color.black);
        chartPanel = new ChartPanel(chart);
        chartPanel.setPreferredSize(new java.awt.Dimension(500, 270));
        setContentPane(chartPanel);
    }
    
    public void showWindow() {
        pack();        
        RefineryUtilities.centerFrameOnScreen(this);
        setVisible(true);
    }

    @Override
    public void update(Sensor source, SensorData sd) {
        if(sd instanceof AccelerationData) {
            AccelerationData ad = (AccelerationData) sd;
            acumulativeTime += ad.getInterval();
           
            xAcceleration.add(acumulativeTime, ad.getX());
            yAcceleration.add(acumulativeTime, ad.getY());
            zAcceleration.add(acumulativeTime, ad.getZ());   
            if (xAcceleration.getItemCount() > datasize) {
            	xAcceleration.remove(0);
            	yAcceleration.remove(0);
            	zAcceleration.remove(0);
            }
            	
            chartPanel.updateUI();
        }
    }

    public void stop() {
    }

    @Override
    public void cleanUp() {
        setVisible(false);
        xAcceleration = null;
        yAcceleration = null;
        zAcceleration = null;
    
        dataset = null;    
        chart = null;
        plot = null;
    
        chartPanel = null;
    }
}