/*
                           KiteModeler
   
                           A Java Applet
               to design and evaluate a variety of kites

                           Version 1.5a

                         Written by Tom Benson
                       NASA Glenn Research Center

>                              NOTICE
>This software is in the Public Domain.  It may be freely copied and used in
>non-commercial products, assuming proper credit to the author is given.  IT
>MAY NOT BE RESOLD.  If you want to use the software for commercial
>products, contact the author.
>No copyright is claimed in the United States under Title 17, U. S. Code.
>This software is provided "as is" without any warranty of any kind, either
>express, implied, or statutory, including, but not limited to, any warranty
>that the software will conform to specifications, any implied warranties of
>merchantability, fitness for a particular purpose, and freedom from
>infringement, and any warranty that the documentation will conform to the
>program, or any warranty that the software will be error free.
>In no event shall NASA be liable for any damages, including, but not
>limited to direct, indirect, special or consequential damages, arising out
>of, resulting from, or in any way connected with this software, whether or
>not based on warranty, contract, tort or otherwise, whether or not injury
>was sustained by persons or property or otherwise, and whether or not loss
>was sustained from, or arose out of the results of, or use of, the software
>or services provided hereunder.
 
  New test -
              * create panels - look like RocketModeler - begun 15Jan02
              * add default geometries - reset button
              * add string tension output
              * add large output box - save geometry (like FoilSim)
                  mods for Langley Mars project
              * add atmosphere data
                add new geometry
              * add new materials -- not necessary - Langley
              * re-arrange layout
              * add some labels
                add a variable to allow very large/small kites ..max dimension
                
                                           TJB  28 Aug 12

*/

import java.awt.*;
import java.lang.Math ;

public class Kite extends java.applet.Applet {
 
   static double convdr = 3.1415926/180. ;
   static double pid2 = 3.1415926/2.0 ;
   static double pi = 3.1415926 ;

   static double ps0,ts0 ;
   static double h1,h2,w1,w2,lkite ;
   static double h1d,h2d,w1d,w2d ;
   static double hmn,hmx,wmn,wmx ;
   static double hmnd,hmxd,wmnd,wmxd ;
   static double area,lengstk,cg,cp,ar;
   static double weight,lift,drag,wtplnt,wtlplnt;
   static double wtarea,wtlngs,wtlngl,wttail,wtrat;
   static double lbrid,lknot,ltail,xbr,ybr,kbase ;
   static double lbridd,lknotd,ltaild ;
   static double lbrmn,lbrmx,ltlmn,ltlmx ;
   static double lbrmnd,lbrmxd,ltlmnd,ltlmxd ;
   static double anga,lpiv,wind,lline,alt,wpay ;
   static double windd,llined,altd,wpayd ;
   static double wndmn,wndmx,llnmn,llnmx,altmn,altmx,wpaymn,wpaymx ;
   static double wndmnd,wndmxd,llnmnd,llnmxd,altmnd,altmxd,wpaymnd,wpaymxd ;
   static double density,cl,cd ;
   static double angl,angk,alpha,torq,tension;
   static double almn,almx ;
   static double lconv,wconv,hconv ;
   static double[] tor  = new double[405] ; 
   static double con1,con2,xkite,ykite;
   static double[] xline = new double[21] ; 
   static double[] yline = new double[21] ; 

   static int surtyp,stktyp,lintyp,taltyp;
   static int iter,unstab,nolift ;

   static int ktype,angmod,units,fold,ground,planet;
   static double beta;

       /*  plot & probe data */
   static double fact,fscale,fscmin,fscrat;
   static double fact0,fact1,fact2,fsc1,fsc2;
   static int xt,yt,sldloc,fldloc,viewflg,pick;
   static int xt0,yt0,sld0;
   static int xt1,yt1,sld1,fld1;
   static int xt2,yt2,sld2,fld2;
 
   Solver solve ;
   Viewer view ;
   C c ;
   CardLayout layin,layout ;
   Image offImg1 ;
   Graphics off1Gg ;
   Image offImg2 ;
   Graphics off2Gg ;

   public void init() {
     int i;
     Kite a = new Kite() ;
     solve = new Solver() ;

     offImg1 = createImage(this.size().width,
                      this.size().height) ;
     off1Gg = offImg1.getGraphics() ;
     offImg2 = createImage(this.size().width,
                      this.size().height) ;
     off2Gg = offImg2.getGraphics() ;

     setLayout(new GridLayout(1,2,5,5)) ;

     solve.setDefaults () ;
 
     view  = new Viewer(this) ;
     c  = new C(this) ;

     add(view) ;
     add(c) ;

     loadInput() ;

     compute () ;
     view.start() ;
  }
 
  public Insets insets() {
     return new Insets(5,5,5,5) ;
  }

  public void compute() { 
     double wt1,wt2,wt3,wtail;
     double cosa,sina,tana,thet;
     double el,bet2;
     double alold,alnew,tnew,told;
     double slope,tzero,cosl,sinl ;
     double a1,a2,a3,c1 ;
     double delx ;
     int isave,iflag ;

     solve.getFreeStream() ;

 // compute geometry for various kites

     wtail = ltail * wttail ;

    if (ktype == 0) { // diamond
       lkite = h1 + h2 ;
       area = .5 * lkite * w1 ;
       wt1 = area * wtarea;
       lengstk = lkite + w1 ;
       wt2 = lkite *  wtlngs ;
       wt3 = w1 *  wtlngs ;
       weight = wt1 + wt2 + wt3 + wtail ;
       cg = (wt1 * (h1 + 2.0* h2) / 3.0 +
             wt2 * lkite / 2.0 +
             wt3 * h2  +
             wtail * (-ltail/2.0)
            )/ weight ;
       cp = .5 * lkite + h2/3.0 ;
       ar = w1 * w1 / area ;
       kbase = 0.0 ;
    }
    if (ktype == 1) { //delta
       lkite = h1 + h2 ;
       area = .5 * lkite * w1 ;
       wt1 = area * wtarea;
       lengstk = Math.sqrt(4.0*h1*h1 + w1*w1) ;
       wt2 = lengstk *  wtlngs ;
       weight = wt1 + wt2 + wtail ;
       cg = (wt1 * (h1 + 2.0* h2) / 3.0 +
             wt2 * (h2 + h1 /2.0) +
             wtail * (-ltail/2.0)
             ) / weight ; 
       cp = .5 * lkite + h2/3.0 ;
       ar = w1 * w1 / area ;
       kbase = 0.0 ;
     }
     if (ktype == 2) { //sled
       lkite = h1 + h2 ;
       area = .5 * lkite * (w1 + w2) ;
       wt1 = .5 * lkite * (w2 - w1) * wtarea ;
       wt2 = lkite * w1 * wtarea ;
       lengstk = 2.0 * lkite ;
       wt3 = lengstk *  wtlngs ;
       weight = wt1 + wt2 + wt3 + wtail ;
       cg = (wt1 * (h1 + 2.0* h2) / 3.0 +
             wt2 * lkite / 2.0 +
             wt3 * lkite / 2.0 +
             wtail * (-ltail/2.0)
            ) / weight ;
       cp = .85*(wt1 * (.5 * lkite + h2/3.0) +
             wt2 * .75 * lkite) / (wt1 + wt2) ;
       ar = w2 * w2 / area ;
       kbase = 0.0 ;
     }
     if (ktype == 3) { //box
       lkite = 2.0*h1 + h2 ;
       area = 4.0 * h1 * w1 ;
       lengstk = 4.0 * lkite + 4.0 * w1 ;
       weight = area * wtarea + lengstk * wtlngs + wtail ;
       cg = ((h1 + .5 * h2) * (weight - wtail) +
              wtail * (-ltail/2.0)
            ) / weight ;
       cp = h1 + .5 * h2 + .25*h1 ;
       ar = w1 / h1 ;
       kbase = w1 / 2.0 ;
     }
     if (ktype == 4) { //winged box
       lkite = 2.0*h1 + h2 ;
       area = 4.0* h1 * w1 + .5 * lkite * (w2 - w1) ;
       wt1 = .5 * lkite * (w2 - w1) * wtarea ;
       lengstk = 4.0 * lkite + 4.0 * w1 + w2 ;
       wt2 = (4.0* h1 * w1) * wtarea + (4.0 * lkite + 4.0 * w1)*wtlngs ;
       wt3 = w2 *  wtlngs ;
       weight = wt1 + wt2 + wt3 + wtail ;
       cg = (wt1 * (h1 + 2.0* h2) / 3.0 +
             wt2 * (h1 + .5 * h2) +
             wt3 * (h1 + h2) +
             wtail * (-ltail/2.0)
            ) / weight ;
       cp = (wt1 * (.75 * lkite - (.75*lkite - (h1+h2))/3.0) +
            ((4.0* h1 * w1) * wtarea)*( h1 + .5 * h2 + .25*h1))/
             (wt1 + ((4.0* h1 * w1) * wtarea)) ;
       ar = w2 / h1 ;
       kbase = w1 / 2.0 ;
     }
    if (ktype == 5) { // twin-trapezoid  -- special design from treacher
       el = w2 / 2.0 - w1 / 2.0 ;
       lkite = Math.sqrt(el * el + h1 * h1) ;
       area = h1 * (h2 - w1 - el) ;
       wt1 = (h1 * h2) * wtarea;
       bet2 = Math.atan(el / h1) / convdr ;
       beta = 90.0 - bet2 ;
       lengstk = (h2 - w1) * Math.sin(convdr*beta) ;
       wt2 = lengstk *  wtlngs ;
       weight = wt1 + wt2 + wtail ;
       cg = (wt1 * lkite / 2.0 +
             wt2 * (lkite - (h2-w1)*Math.cos(convdr*beta)) +
             wtail * (-ltail/2.0)
            )/ weight ;
 // look at Cp carefully
       cp = .75 * lkite - (h2-w1)*Math.cos(convdr*beta)/2.0 ;
       ar = lengstk * lengstk / area ;
       kbase = 0.0 ;
     }
     if (ktype == 6) { //  tumbleweed
       lkite = 2.0 * h1 ;
       area = pi * h1 * h1 ;
       lengstk = w1 * 2.0 * pi * h1 ;
       wt1 = 3.0 * area * wtarea + lengstk * wtlngs ;
       weight = wt1 + wtail ;
       cg = (h1 * wt1 +
             wtail * (-ltail/2.0)
             )/weight ;
       cp = h1 + .25*h1 ;
       ar = 1.0 ;
       kbase = 0.0 ;
     }
        
 //  adjust weight for different planet gravity and payload
    wtplnt = (weight + wpay) * wtrat ;
    wtlplnt = wtlngl * wtrat ;
 
 // compute the bridle geometry - bridle point

     if (ktype == 2) { // special for sled
         if (lbrid < w2) lbrid = w2 + .01 ;
         lknot = lbrid ;

         ybr = h2 ;
         tana = lbrid /(w2 - w1) ;
         cosa = w1 / Math.sqrt(lbrid*lbrid + (w2-w1)*(w2-w1)) ;
         thet = 3.1415926 - Math.atan(tana) - Math.acos(cosa) ;
         xbr = .5*(w2 - w1) * Math.sin(thet) ; 
     }
     if (ktype == 5) {  // special for twintrap
         if (lknot > lkite) lknot = lkite - .1 ;
         ybr = lknot ;
         xbr = .4 ;
     }
     if (ktype == 0 || ktype == 1 || ktype == 3 || ktype == 4) { // others
         if (lbrid < lkite) lbrid = lkite + .01 ;
         if (lknot > lbrid) lknot = lbrid ;

         cosa = (lknot*lknot + lkite*lkite - (lbrid-lknot)*(lbrid-lknot))/
                (2.0 * lknot * lkite) ;
         anga = Math.acos(cosa) ;
         sina = Math.sin(anga) ;
         xbr = lknot * sina ;
         ybr = lknot * cosa ;
         xbr = xbr + kbase;
     }
     if (ktype == 6) {  // special for tumbleweed
         anga = lbrid * convdr ;
         xbr = h1 * Math.sin(lbrid * convdr)  ;
         ybr = h1 + h1 * Math.cos(lbrid * convdr) ;
     }

  // determine trim angle

     if (angmod > 0) { // angle input mode
        iter = 0;
        unstab = 0 ;
        c.out.dn.des.o9.setForeground(Color.yellow) ;
        c.out.dn.des.o10.setForeground(Color.yellow) ;
        c.out.dn.flt.o9.setForeground(Color.yellow) ;
        c.out.dn.flt.o10.setForeground(Color.yellow) ;
        alnew = angk =  alpha * convdr ;
        cosl = Math.cos(alnew) ;
        sinl = Math.sin(alnew) ;

        cl = 2.0 * pi * alnew / (1.0 + 2.0 * alnew / ar) ;
        cd = 1.28 * sinl + (cl*cl)/(.7 * pi * ar) ;
 
        if (ktype == 6) cd = cd + 1.28 * cosl ;

        lift = .5 * cl * density * wind * wind * area / 144.0 * 16.0 ;
        drag = .5 * cd * density * wind * wind * area / 144.0 * 16.0 ;
   
        torq = - lift * cosl * (ybr - cp) 
               - lift * sinl * xbr
               + drag * cosl * xbr
               - drag * sinl * (ybr - cp) 
               + wtplnt * cosl * (ybr - cg)
               + wtplnt * sinl * xbr ;
        if (lift > wtplnt) {
            angl = Math.atan(drag / (lift - wtplnt)) ;
            nolift = 0 ;
        }
        else {
            angl = 0.0 ;
            nolift = 1 ;
        }
     }
 
     if (angmod == 0) { // compute angle

    // load array for range of angles
        c.out.dn.des.o9.setForeground(Color.green) ;
        c.out.dn.des.o10.setForeground(Color.green) ;
        c.out.dn.flt.o9.setForeground(Color.green) ;
        c.out.dn.flt.o10.setForeground(Color.green) ;
        for(iter = 0; iter <=400; ++iter){
           alnew = iter* .125 * convdr ;
           cosl = Math.cos(alnew) ;
           sinl = Math.sin(alnew) ;

           cl = 2.0 * pi * alnew / (1.0 + 2.0 * alnew / ar) ;
           cd = 1.28 * sinl + (cl*cl)/(.7 * pi * ar) ;

           if (ktype == 6) cd = cd + 1.28 * cosl ;

           lift = .5 * cl * density * wind * wind * area / 144.0 * 16.0 ;
           drag = .5 * cd * density * wind * wind * area / 144.0 * 16.0 ;
   
           tor[iter] = - lift * cosl * (ybr - cp) 
                       - lift * sinl * xbr
                       + drag * cosl * xbr
                       - drag * sinl * (ybr - cp) 
                       + wtplnt * cosl * (ybr - cg)
                       + wtplnt * sinl * xbr ;
        }

   // check for stability -- find where torque sign changes 
        unstab = 1 ;
        iflag = 0 ;
        isave = 0 ;
        for(iter = 0; iter <=400; ++iter){
           if(tor[iter] < 0.0) {
              unstab = 0 ;
              if (iflag == 0) {
                 iflag = 1 ;
                 isave = iter ;
              }
              if (isave == 0) {
                 unstab = 1 ;
                 iflag = 0 ;
              }
           }
        }
    // load answer
        if (unstab == 0) {
           alpha = isave * .125 ;
           angk = alpha * convdr ;
           cosl = Math.cos(angk) ;
           sinl = Math.sin(angk) ;

           cl = 2.0 * pi * angk / (1.0 + 2.0 * angk / ar) ;
           cd = 1.28 * sinl + (cl*cl)/(.7 * pi * ar) ;

           if (ktype == 6) cd = cd + 1.28 * cosl ;

           lift = .5 * cl * density * wind * wind * area / 144.0 * 16.0 ;
           drag = .5 * cd * density * wind * wind * area / 144.0 * 16.0 ;
   
           torq = - lift * cosl * (ybr - cp) 
                  - lift * sinl * xbr
                  + drag * cosl * xbr
                  - drag * sinl * (ybr - cp) 
                  + wtplnt * cosl * (ybr - cg)
                  + wtplnt * sinl * xbr ;
           if (lift > wtplnt) {
               angl = Math.atan(drag / (lift - wtplnt)) ;
               nolift = 0 ;
           }
           else {
               angl = 0.0 ;
               nolift = 1 ;
           }
        }
     }

     if (lift > wtplnt) nolift = 0 ;
     else nolift = 1 ;

     if (nolift == 1) unstab = 0 ;

  // compute the kite altitude

     ground = 0 ;
     a1 = (lift - wtplnt - lline*wtlplnt);
     c1 = drag / wtlplnt ;
     if (a1 <= 0.0 ) {
         a1 = 0.0 ;
         ground = 1;
     }
     con1 = isinh(a1/drag) ;
     con2 = -c1 * cosh(con1) ;

     a2 = (lift - wtplnt) / drag ;
     if (a2 <= 0.0 ) a2 = 0.0 ;
     a3 = isinh(a2) ;
     xkite = c1 * (a3 - con1) ;
/*
 c.in.flt.l.diag1.setText(String.valueOf(filter3(xkite))) ;
 c.in.flt.l.diag2.setText(String.valueOf(filter3(c1))) ;
 c.in.flt.l.diag3.setText(String.valueOf(filter3(a3))) ;
 c.in.flt.l.diag4.setText(String.valueOf(filter3(con1))) ;
*/
     if (c1 > 0.0) {
       ykite = con2 + c1 * cosh((xkite/c1) + con1) ;
     }
     else {
       ykite = 0.0 ;
     }

  // compute the line shape
 
     delx = xkite / 20.0 ;
     for(iter = 0; iter <=20; ++iter){
        xline[iter] = iter * delx ;
        if (c1 > 0.0) {
           yline[iter] = con2 + c1 * cosh((xline[iter]/c1) + con1) ;
        }
        else {
          ykite = 0.0 ;
        }
     }

  // compute the line tension

     tension = Math.sqrt(a1 * a1 + drag * drag) ;

     loadOut() ;
  }

  public int filter0(double inumbr) {
        //  output only to .
       int number ;
       int intermed ;
 
       number = (int) (inumbr);
       return number ;
  }

  public float filter1(double inumbr) {
     //  output only to .1
       float number ;
       int intermed ;
 
       intermed = (int) (inumbr * 10.) ;
       number = (float) (intermed / 10. );
       return number ;
  }
 
  public float filter3(double inumbr) {
     //  output only to .001
       float number ;
       int intermed ;
 
       intermed = (int) (inumbr * 1000.) ;
       number = (float) (intermed / 1000. );
       return number ;
  }
 
  public float filter5(double inumbr) {
     //  output only to .00001
       float number ;
       int intermed ;
 
       intermed = (int) (inumbr * 100000.) ;
       number = (float) (intermed / 100000. );
       return number ;
  }
 
  public void setFrontView() {   // initial front view
       viewflg = 0 ;
       fact = fact0 = 10.0;
       xt = xt0 = 150;
       yt = yt0 = 225 ;
       sldloc = sld0 = 70;
  }

  public void restFrontView() {   // restore front view
       viewflg = 0 ;
       fact = fact0 ;
       xt = xt0 ;
       yt = yt0 ;
       sldloc = sld0 ;
  }

  public void setSideView() {   // initial side view
       viewflg = 1 ;
       fact = fact1 = 10.0;
       xt = xt1 = 200;
       yt = yt1 = 225 ;
       sldloc = sld1 = 70;
       fscmin = 1.0 ; fscrat = .3;
       fscale =  fsc1 = 10.0 ;
       fldloc =  fld1 = 150 ;
  }

  public void restSideView() {   // restore side view
       viewflg = 1 ;
       fact = fact1 ;
       xt = xt1 ;
       yt = yt1 ;
       sldloc = sld1 ;
       fscmin = 1.0 ; fscrat = .3;
       fscale =  fsc1 ;
       fldloc =  fld1 ;
  }

  public void setFieldView() {   // initial field view
       viewflg = 2 ;
       fact = fact2 = 10.0;
       xt = xt2 = 50;
       yt = yt2 = 300 ;
       sldloc = sld2 = 70;
       fscmin = .04 ; fscrat = .007;
       fscale =  fsc2 = .25 ;
       fldloc =  fld2 = 150 ;
  }

  public void restFieldView() {   // restore field view
       viewflg = 2 ;
       fact = fact2 ;
       xt = xt2 ;
       yt = yt2 ;
       sldloc = sld2 ;
       fscmin = .04 ; fscrat = .007;
       fscale =  fsc2 ;
       fldloc =  fld2 ;
  }

  public double cosh(double inumbr) {
     //  hyperbolic cosine
       double number ;
 
       number = 1.0 
              + Math.pow(inumbr,2)/2.0 
              + Math.pow(inumbr,4)/24.0
              + Math.pow(inumbr,6)/720.0 
              + Math.pow(inumbr,8)/40320. ;
       return number ;
  }
 
  public double isinh(double inumbr) {
     //  inverse hyperbolic sine
       double number ;
 
       if (inumbr * inumbr <= 1.0) {
          number = inumbr 
                 - Math.pow(inumbr,3)/6.0 
                 + 3.0  * Math.pow(inumbr,5)/40.0
                 - 15.0 * Math.pow(inumbr,7)/336. 
                 + 21.0 * Math.pow(inumbr,9)/3456. ; 
       }
       else {
          number = Math.log(2.0*inumbr) 
                 + 1.0   / (4.0    * Math.pow(inumbr,2)) 
                 - 3.0   / (32.0   * Math.pow(inumbr,4)) 
                 + 15.0  / (288.0  * Math.pow(inumbr,6))
                 - 105.0 / (3072.0 * Math.pow(inumbr,8)) ;
       }
       return number ;
  }
 
  public void loadInput() {   // load the input panels
       int i1,i2,i3,i4,i5,i6 ;
       double v1,v2,v3,v4,v5,v6 ;
       float fl1,fl2,fl3,fl4,fl5,fl6 ;

       h1d = h1 * lconv ;
       h2d = h2 * lconv ;
 
       if (ktype < 6) {
          w1d = w1 * lconv ;
          w2d = w2 * lconv ;
          fl3 = (float) w1d ;
          fl4 = (float) w2d ;
          wmnd = wmn * lconv ;
          wmxd = wmx * lconv ;
          i3 = (int) (((w1d - wmnd)/(wmxd-wmnd))*1000.) ;
          i4 = (int) (((w2d - wmnd)/(wmxd-wmnd))*1000.) ;
          c.in.shp.l.f3.setText(String.valueOf(fl3)) ;
          c.in.shp.l.f4.setText(String.valueOf(fl4)) ;
          c.in.shp.r.s3.setValue(i3) ;
          c.in.shp.r.s4.setValue(i4) ;
       }

       fl1 = (float) h1d ;
       fl2 = (float) h2d ;

       hmnd = hmn * lconv ;
       hmxd = hmx * lconv ;

       i1 = (int) (((h1d - hmnd)/(hmxd-hmnd))*1000.) ;
       i2 = (int) (((h2d - hmnd)/(hmxd-hmnd))*1000.) ;

       c.in.shp.l.f1.setText(String.valueOf(fl1)) ;
       c.in.shp.l.f2.setText(String.valueOf(fl2)) ;

       c.in.shp.r.s1.setValue(i1) ;
       c.in.shp.r.s2.setValue(i2) ;

       if (ktype < 5) {
         if (lbrid < lkite) lbrid = lkite + .01 ;
         if (lknot > lbrid) lknot = lbrid ;
       }

       if (ktype < 6) {
         lbridd = lbrid * lconv ;
         fl1 = (float) lbridd ;
         lbrmnd = lbrmn * lconv ;
         lbrmxd = lbrmx * lconv ;
         i1 = (int) (((lbridd - lbrmnd)/(lbrmxd-lbrmnd))*1000.) ;
         c.in.trm.l.f1.setText(String.valueOf(fl1)) ;
         c.in.trm.r.s1.setValue(i1) ;
       }

       lknotd = lknot * lconv ;
       ltaild = ltail * lconv ;

       fl2 = (float) lknotd ;
       fl3 = (float) alpha ;
       fl4 = (float) ltaild ;

       ltlmnd = ltlmn * lconv ;
       ltlmxd = ltlmx * lconv ;

       i2 = (int) (((lknotd - lbrmnd)/(lbrmxd-lbrmnd))*1000.) ;
       i3 = (int) (((alpha - almn)/(almx-almn))*1000.) ;
       i4 = (int) (((ltaild - ltlmnd)/(ltlmxd-ltlmnd))*1000.) ;

       c.in.trm.l.f2.setText(String.valueOf(fl2)) ;
       c.in.trm.l.f3.setText(String.valueOf(fl3)) ;
       c.in.trm.l.f4.setText(String.valueOf(fl4)) ;

       c.in.trm.r.s2.setValue(i2) ;
       c.in.trm.r.s3.setValue(i3) ;
       c.in.trm.r.s4.setValue(i4) ;

       windd = wind * hconv ;
       altd = alt * hconv ;
       llined = lline * hconv ;
       wpayd = wpay * wconv ;

       fl1 = (float) windd ;
       fl2 = (float) altd ;
       fl3 = (float) llined ;
       fl4 = (float) wpayd ;

       wndmnd = wndmn * hconv ;
       wndmxd = wndmx * hconv ;
       altmnd = altmn * hconv ;
       altmxd = altmx * hconv ;
       llnmnd = llnmn * hconv ;
       llnmxd = llnmx * hconv ;
       wpaymnd = wpaymn * wconv ;
       wpaymxd = wpaymx * wconv ;

       i1 = (int) (((windd - wndmnd)/(wndmxd-wndmnd))*1000.) ;
       i2 = (int) (((altd - altmnd)/(altmxd-altmnd))*1000.) ;
       i3 = (int) (((llined - llnmnd)/(llnmxd-llnmnd))*1000.) ;
       i4 = (int) (((wpayd - wpaymnd)/(wpaymxd-wpaymnd))*1000.) ;

       c.in.flt.l.f1.setText(String.valueOf(fl1)) ;
       c.in.flt.l.f2.setText(String.valueOf(fl2)) ;
       c.in.flt.l.f3.setText(String.valueOf(fl3)) ;
       c.in.flt.l.f4.setText(String.valueOf(fl4)) ;

       c.in.flt.r.s1.setValue(i1) ;
       c.in.flt.r.s2.setValue(i2) ;
       c.in.flt.r.s3.setValue(i3) ;
       c.in.flt.r.s4.setValue(i4) ;

       fl1 = (float) (wtarea * wconv / lconv / lconv) ;
       fl2 = (float) (wtlngs * wconv / lconv)  ;
       fl3 = (float) (wtlngl * wconv / hconv)  ;
       fl4 = (float) (wttail * wconv / lconv)  ;
       fl5 = (float) (hmx * lconv)  ;

       c.in.mat.f1.setText(String.valueOf(fl1)) ;
       c.in.mat.f2.setText(String.valueOf(fl2)) ;
       c.in.mat.f3.setText(String.valueOf(fl3)) ;
       c.in.mat.f4.setText(String.valueOf(fl4)) ;
       c.in.mat.f5.setText(String.valueOf(fl5)) ;

       return ;
  }

  public void loadOut() {   // output routine
     int i3 ;
     double v3 ;
     float fl3 ;
 
     c.out.up.o3.setText(String.valueOf(filter3(wtplnt * wconv))) ;
     c.out.up.o7.setText(String.valueOf(filter3(lift * wconv))) ;
     c.out.up.o8.setText(String.valueOf(filter3(drag * wconv))) ;
     c.out.up.o17.setText(String.valueOf(filter3(tension * wconv))) ;

     c.out.dn.des.o1.setText(String.valueOf(filter3(area *lconv * lconv))) ;
     c.out.dn.des.o2.setText(String.valueOf(filter3(lengstk *lconv))) ;
     c.out.dn.des.o4.setText(String.valueOf(filter3(cg *lconv))) ;
     c.out.dn.des.o5.setText(String.valueOf(filter3(cp *lconv))) ;
     c.out.dn.des.o9.setText(String.valueOf(filter3(torq))) ;
     c.out.dn.des.o10.setText(String.valueOf(filter3(alpha))) ;

     c.out.dn.flt.o12.setText(String.valueOf(filter0(xkite * hconv))) ;
     c.out.dn.flt.o13.setText(String.valueOf(filter0(ykite * hconv))) ;
     c.out.dn.flt.o9.setText(String.valueOf(filter3(torq))) ;
     c.out.dn.flt.o10.setText(String.valueOf(filter3(alpha))) ;
 
     if (units == 1) {
        c.out.dn.flt.o14.setText(String.valueOf(filter3(ps0 / 144.))) ;
        c.out.dn.flt.o15.setText(String.valueOf(filter0(ts0 - 460.))) ;
     }
     if (units == -1) {
        c.out.dn.flt.o14.setText(String.valueOf(filter3(101.3/14.7*ps0/144.))) ;
        c.out.dn.flt.o15.setText(String.valueOf(filter3(ts0*5.0/9.0 - 273.1))) ;
     }

     return ;
  }
 
  public void printData() {
     String outlng,outln2,outfor,outdis ;

     outlng = " in" ;
     if (units == -1) outlng = " cm" ;
     outln2 = " sq in" ;
     if (units == -1) outln2 = " sq cm" ;
     outfor = " oz" ;
     if (units == -1) outfor = " gm" ;
     outdis = " ft" ;
     if (units == -1) outdis = " m" ;

     switch(ktype) {
        case 0: {
           c.in.prt.prnt.appendText( "\n\n Diamond Kite - " ) ;
           break ;
        }
        case 1: {
           c.in.prt.prnt.appendText( "\n\n Delta Kite - " ) ;
           break ;
        }
        case 2: {
           c.in.prt.prnt.appendText( "\n\n Sled Kite - " ) ;
           break ;
        }
        case 3: {
           c.in.prt.prnt.appendText( "\n\n Box Kite - " ) ;
           break ;
        }
        case 4: {
           c.in.prt.prnt.appendText( "\n\n Winged Box - " ) ;
           break ;
        }
        case 5: {
           c.in.prt.prnt.appendText( "\n\n Twin-Trap - " ) ;
           break ;
        }
        case 6: {
           c.in.prt.prnt.appendText( "\n\n Tumbleweed - " ) ;
           break ;
        }
     }
     switch(taltyp) {   /// tail material
        case 0: {
           c.in.prt.prnt.appendText( " 1 inch Plastic Tail," ) ;
           break ;
        }
        case 1: {
           c.in.prt.prnt.appendText( " 3 inch Plastic Tail," ) ;
           break ;
        }
        case 2: {
           c.in.prt.prnt.appendText( " 1 inch Nylon Tail," ) ;
           break ;
        }
        case 3: {
           c.in.prt.prnt.appendText( " Specified Tail," ) ;
           break ;
        }
     }
     switch(lintyp) {   /// line material
        case 0: {
           c.in.prt.prnt.appendText( " Nylon Line" ) ;
           break ;
        }
        case 1: {
           c.in.prt.prnt.appendText( " Cotton Line" ) ;
           break ;
        }
        case 2: {
           c.in.prt.prnt.appendText( " Specified Line" ) ;
           break ;
        }
     }

     switch(surtyp) {    // surface  material
        case 0: {
           c.in.prt.prnt.appendText( "\n Plastic Surface," ) ;
           break ;
        }
        case 1: {
           c.in.prt.prnt.appendText( "\n Tissue Surface," ) ;
           break ;
        }
        case 2: {
           c.in.prt.prnt.appendText( "\n Rip Stop Surface," ) ;
           break ;
        }
        case 3: {
           c.in.prt.prnt.appendText( "\n Paper Surface," ) ;
           break ;
        }
        case 4: {
           c.in.prt.prnt.appendText( "\n Silk Span Surface," ) ;
           break ;
        }
        case 5: {
           c.in.prt.prnt.appendText( "\n Specified Surface," ) ;
           break ;
        }
     }
     switch(stktyp) {         // frame material
        case 0: {
           c.in.prt.prnt.appendText( " 1/4 Balsa Frame," ) ;
           break ;
        }
        case 1: {
           c.in.prt.prnt.appendText( " 1/8 Blasa Frame," ) ;
           break ;
        }
        case 2: {
           c.in.prt.prnt.appendText( " 1/4 Birch Frame," ) ;
           break ;
        }
        case 3: {
           c.in.prt.prnt.appendText( " 3/8 Plastic Frame," ) ;
           break ;
        }
        case 4: {
           c.in.prt.prnt.appendText( " Skewer Frame," ) ;
           break ;
        }
        case 5: {
           c.in.prt.prnt.appendText( " Specified Frame," ) ;
           break ;
        }
     }
     c.in.prt.prnt.appendText( "\n Tail = " + filter1(ltaild) + outlng ) ;
     c.in.prt.prnt.appendText( ",Line = " + filter0(llined) + outdis ) ;
     c.in.prt.prnt.appendText( ",Weight = " + filter3(weight*wconv) + outfor ) ;

     c.in.prt.prnt.appendText( "\n Lift = " + filter3(lift*wconv)) ;
     c.in.prt.prnt.appendText( ",Drag = " + filter3(drag*wconv)) ;
     c.in.prt.prnt.appendText( ",Tension = " + filter3(tension*wconv) + outfor ) ;

     c.in.prt.prnt.appendText( "\n H1 = " + filter1(h1d)) ;
     c.in.prt.prnt.appendText( ",H2 = " + filter1(h2d)) ;
     c.in.prt.prnt.appendText( ",W1 = " + filter1(w1d)) ;
     c.in.prt.prnt.appendText( ",W2 = " + filter1(w2d) + outlng ) ;
     c.in.prt.prnt.appendText( "\n Surface = " + filter1(area*lconv*lconv) + outln2);
     c.in.prt.prnt.appendText( " Frame = " + filter1(lengstk*lconv) + outlng ) ;

     c.in.prt.prnt.appendText( "\n Cg = " + filter3(cg*lconv) + outlng ) ;
     c.in.prt.prnt.appendText( ",Cp = " + filter3(cp*lconv) + outlng ) ;
     c.in.prt.prnt.appendText( ",AR = " + filter3(ar) ) ;

     c.in.prt.prnt.appendText( "\n Bridle = " + filter1(lbridd) + outlng ) ;
     c.in.prt.prnt.appendText( " Knot = " + filter1(lknotd) + outlng ) ;
     c.in.prt.prnt.appendText( "\n Trim Angle = " + filter3(alpha) ) ;
     c.in.prt.prnt.appendText( ",Torque = " + filter3(torq*wconv*lconv) ) ;
     if (units == 1) c.in.prt.prnt.appendText( " oz-in," ) ;
     if (units == -1) c.in.prt.prnt.appendText( " gm-cm," ) ;

     switch(planet) {         //atmospherics
        case 0: {
           c.in.prt.prnt.appendText( "\n Earth" ) ;
           break ;
        }
        case 1: {
           c.in.prt.prnt.appendText( "\n Mars" ) ;
           break ;
        }
     }
     c.in.prt.prnt.appendText( ",Alt = " + filter0(alt*hconv)) ;
     c.in.prt.prnt.appendText( ",Wind = " + filter1(wind*hconv) ) ;
     if (units == 1) c.in.prt.prnt.appendText( " ft/sec," ) ;
     if (units == -1) c.in.prt.prnt.appendText( " m/sec," ) ;

     c.in.prt.prnt.appendText( "\n Range = " + filter0(xkite*hconv)) ;
     c.in.prt.prnt.appendText( ",Height = " + filter0(ykite*hconv) + outdis ) ;
  }

  class Solver {
 
     Solver () {
     }

     public void setDefaults() {

        ktype = 0;

        h1d = h1 = 5.0 ;
        h2d = h2 = 10.0 ;
        w1d = w1 = 10.0 ;
        w2d = w2 = 20.0 ;
        lkite = 15. ;
        kbase = 0.0 ;
        area = 75.0;
        lengstk = 25.0 ;
        hmnd = hmn = .5 ;  hmxd = hmx = 120. ;
        wmnd = wmn = .5 ;  wmxd = wmx = 120. ;
        surtyp = 0; wtarea = .0004752 ; 
        stktyp = 0; wtlngs = .003952; 
        lintyp = 0; wtlngl=.004;
        taltyp = 0; wttail=.0004752;

        lbridd = lbrid = 15.5 ;
        lknotd = lknot = 12. ;
        angmod = 0 ;
        alpha = 5.0 ;
        ltaild = ltail = 12.0 ;
        llined = lline = 100. ;
        windd = wind = 10.0 ;
        lbrmnd = lbrmn = 0.0 ; lbrmxd = lbrmx = 250. ; 
        ltlmnd = ltlmn = 0.0 ; ltlmxd = ltlmx = 60. ;
        llnmnd = llnmn = 0.0 ; llnmxd = llnmx = 5000. ;
        wndmnd = wndmn = 0.5 ; wndmxd = wndmx = 50. ;
        almn = 0.0 ; almx = 45.0 ;
        planet = 0 ;
        altd = alt = 0.0 ;
        altmnd = altmn = 0.0 ; altmxd = altmx = 100000. ;
        wpayd = wpay = 0.0 ;
        wpaymnd = wpaymn = 0.0 ; wpaymxd = wpaymx = 160. ;
        density = .00237 ;

        setFieldView () ;
        setSideView () ;
        setFrontView () ;
        pick = -1 ;

        units = 1 ;
        lconv = 1.0 ;
        wconv = 1.0 ;
        hconv = 1.0 ;
        wtrat = 1.0 ;

        fold = -1 ;
        ground = 0 ;
        return ;
     }
 
     public void getFreeStream() {    //  free stream conditions
       double hite,rgas,gama ;       /* MODS  19 Jan 00  whole routine*/

       rgas = 1718. ;                /* ft2/sec2 R */
       gama = 1.4 ;
       hite = alt ;
       wtrat = 1.0 ;
       if (planet == 0) {    // Earth  standard day
         if (hite <= 36152.) {           // Troposphere
            ts0 = 518.6 - 3.56 * hite/1000. ;
            ps0 = 2116. * Math.pow(ts0/518.6,5.256) ;
         }
         if (hite >= 36152. && hite <= 82345.) {   // Stratosphere
            ts0 = 389.98 ;
            ps0 = 2116. * .2236 *
                 Math.exp((36000.-hite)/(53.35*389.98)) ;
         }
         if (hite >= 82345.) {
            ts0 = 389.98 + 1.645 * (hite-82345)/1000. ;
            ps0 = 2116. *.02456 * Math.pow(ts0/389.98,-11.388) ;
         }
       }

       if (planet == 1) {   // Mars - curve fit of orbiter data
         rgas = 1149. ;                /* ft2/sec2 R */
         gama = 1.29 ;
         wtrat = .38 ;

         if (hite <= 22960.) {
            ts0 = 434.02 - .548 * hite/1000. ;
            ps0 = 14.62 * Math.pow(2.71828,-.00003 * hite) ;
         }
         if (hite > 22960.) {
            ts0 = 449.36 - 1.217 * hite/1000. ;
            ps0 = 14.62 * Math.pow(2.71828,-.00003 * hite) ;
         }
       }

       density = ps0/(rgas * ts0) ;

       return ;
     }
  }

  class C extends Panel {
     Kite outerparent ;
     In in ;
     Out out ;

     C (Kite target) { 
       outerparent = target ;
       setLayout(new GridLayout(2,1,5,5)) ;

       in = new In(outerparent) ;
       out = new Out(outerparent) ;

       add(in) ;
       add(out) ;
     }

     class In extends Panel {
        Kite outerparent ;
        Shp shp;
        Mat mat ;
        Flt flt ;
        Trm trm ;
        Prt prt ;
  
        In (Kite target) { 
          outerparent = target ;
          layin = new CardLayout() ;
          setLayout(layin) ;
 
          shp = new Shp(outerparent) ;
          mat = new Mat(outerparent) ;
          flt = new Flt(outerparent) ;
          trm = new Trm(outerparent) ;
          prt = new Prt(outerparent) ;
 
          add ("first", shp) ;
          add ("second", mat) ;
          add ("third", flt) ;
          add ("fourth", trm) ;
          add ("fifth", prt) ;
        }

        class Shp extends Panel {
           Kite outerparent ;
           L l ;
           R r ;

           Shp (Kite target) {

              outerparent = target ;
              setLayout(new GridLayout(1,2,5,5)) ;

              l = new L(outerparent) ;
              r = new R(outerparent) ;

              add(l) ;
              add(r) ;
           }

           class L extends Panel {
              Kite outerparent ;
              TextField f1,f2,f3,f4 ;
              Button bt1,bt2 ;
              Label l1,l2,l3,l4 ;

              L (Kite target) {

               outerparent = target ;
               setLayout(new GridLayout(5,2,2,10)) ;

               bt1 = new Button("Shape") ;
               bt1.setBackground(Color.yellow) ;
               bt1.setForeground(Color.blue) ;

               bt2 = new Button("Material") ;
               bt2.setBackground(Color.white) ;
               bt2.setForeground(Color.blue) ;

               l1 = new Label("H1 - in", Label.CENTER) ;
               f1 = new TextField("5.0",5) ;

               l2 = new Label("H2 - in", Label.CENTER) ;
               f2 = new TextField("10.0",5) ;

               l3 = new Label("W1 - in", Label.CENTER) ;
               f3 = new TextField("10.0",5) ;

               l4 = new Label("W2 - in", Label.CENTER) ;
               f4 = new TextField("20.0",5) ;

               add(bt1) ;
               add(bt2) ;
 
               add(l1) ;
               add(f1) ;
  
               add(l2) ;
               add(f2) ;
  
               add(l3) ;
               add(f3) ;
  
               add(l4) ;
               add(f4) ;
            }
  
            public boolean action(Event evt, Object arg) {
                    if(evt.target instanceof Button) {
                       this.handleDispla(arg) ;
                       return true ;
                    }
                    if(evt.id == Event.ACTION_EVENT) {
                       this.handleText(evt) ;
                       return true ;
                    }
                    else return false ;
                }

                public void handleText(Event evt) {
                  Double V1,V2,V3,V4 ;
                  double v1,v2,v3,v4 ;
                  float fl1 ;
                  int i1,i2,i3,i4 ;
   
                  V1 = Double.valueOf(f1.getText()) ;
                  v1 = V1.doubleValue() ;
                  V2 = Double.valueOf(f2.getText()) ;
                  v2 = V2.doubleValue() ;
                  V3 = Double.valueOf(f3.getText()) ;
                  v3 = V3.doubleValue() ;
                  V4 = Double.valueOf(f4.getText()) ;
                  v4 = V4.doubleValue() ;
   
                  h1d = v1 ;
                  if(h1d < hmnd) {
                    h1d = v1 = hmnd ;
                    fl1 = (float) v1 ;
                    f1.setText(String.valueOf(fl1)) ;
                  }
                  if(h1d > hmxd) {
                    h1d = v1 = hmxd ;
                    fl1 = (float) v1 ;
                    f1.setText(String.valueOf(fl1)) ;
                  }
                  h1 = h1d / lconv ;
 
                  h2d = v2 ;
                  if(h2d < hmnd) {
                    h2d = v2 = hmnd ;
                    fl1 = (float) v2 ;
                    f2.setText(String.valueOf(fl1)) ;
                  }
                  if(h2d > hmxd) {
                    h2d = v2 = hmxd ;
                    fl1 = (float) v2 ;
                    f2.setText(String.valueOf(fl1)) ;
                  }
                  h2 = h2d / lconv ;
 
                  w1d = v3 ;
                  if (ktype == 5) {
                     if(w1d > h2d) {
                       w1d = v3 = h2d - .1 ;
                       fl1 = (float) v3 ;
                       f3.setText(String.valueOf(fl1)) ;
                     }
                  }
                  if(w1d < wmnd) {
                    w1d = v3 = wmnd ;
                    fl1 = (float) v3 ;
                    f3.setText(String.valueOf(fl1)) ;
                  }
                  if(w1d > wmxd) {
                    w1d = v3 = wmxd ;
                    fl1 = (float) v3 ;
                    f3.setText(String.valueOf(fl1)) ;
                  }
                  if(w1d > w2d) {
                    w2d = v4 = w1d +.2 ;
                    fl1 = (float) v4 ;
                    f4.setText(String.valueOf(fl1)) ;
                  }
                  if (ktype < 6) w1 = w1d / lconv ;
                  if (ktype == 6) w1 = w1d ;
  
                  w2d = v4 ;
                  if (ktype == 5) {
                     if(w2d > h2d) {
                       w2d = v3 = h2d - .1 ;
                       fl1 = (float) v3 ;
                       f3.setText(String.valueOf(fl1)) ;
                     }
                  }
                  if(w2d < wmnd) {
                    w2d = v4 = wmnd ;
                    fl1 = (float) v4 ;
                    f4.setText(String.valueOf(fl1)) ;
                  }
                  if(w2d > wmxd) {
                    w2d = v4 = wmxd ;
                    fl1 = (float) v4 ;
                    f4.setText(String.valueOf(fl1)) ;
                  }
                  w2 = w2d / lconv ;
   
                  i1 = (int) (((v1 - hmnd)/(hmxd-hmnd))*1000.) ;
                  i2 = (int) (((v2 - hmnd)/(hmxd-hmnd))*1000.) ;
                  i3 = (int) (((v3 - wmnd)/(wmxd-wmnd))*1000.) ;
                  i4 = (int) (((v4 - wmnd)/(wmxd-wmnd))*1000.) ;
   
                  r.s1.setValue(i1) ;
                  r.s2.setValue(i2) ;
                  r.s3.setValue(i3) ;
                  r.s4.setValue(i4) ;
   
                  compute();
                } // TextHandler

                public void handleDispla(Object arg) {
                  String label = (String)arg ;
 
                  if (label.equals("Shape")) {
                     layin.show(in, "first")  ;
                  }

                  if (label.equals("Material")) {
                     layin.show(in, "second")  ;
                  }
                }  // Button handler
              }  // Inleft
 
              class R extends Panel {
                 Kite outerparent ;
                 Scrollbar s1,s2,s3,s4;
                 Choice ktch;
   
                 R (Kite target) {
                  int i1,i2,i3,i4 ;
   
                  outerparent = target ;
                  setLayout(new GridLayout(5,1,2,10)) ;
   
                  ktch = new Choice() ;
                  ktch.addItem("Diamond Kite") ;
                  ktch.addItem("Delta Kite");
                  ktch.addItem("Sled Kite");
                  ktch.addItem("Box Kite");
                  ktch.addItem("Winged Box Kite");
                  ktch.addItem("Twin-Trap Kite");
                  ktch.addItem("Tumbleweed");
                  ktch.setBackground(Color.white) ;
                  ktch.setForeground(Color.red) ;
                  ktch.select(0) ;

                  i1 = (int) (((5.0 - hmn)/(hmx-hmn))*1000.) ;
                  i2 = (int) (((10.0 - hmn)/(hmx-hmn))*1000.) ;
                  i3 = (int) (((10.0 - wmn)/(wmx-wmn))*1000.) ;
                  i4 = (int) (((20.0 - wmn)/(wmx-wmn))*1000.) ;
   
                  s1 = new Scrollbar(Scrollbar.HORIZONTAL,i1,10,0,1000);
                  s2 = new Scrollbar(Scrollbar.HORIZONTAL,i2,10,0,1000);
                  s3 = new Scrollbar(Scrollbar.HORIZONTAL,i3,10,0,1000);
                  s4 = new Scrollbar(Scrollbar.HORIZONTAL,i4,10,0,1000);
   
                  add(ktch) ;
                  add(s1) ;
                  add(s2) ;
                  add(s3) ;
                  add(s4) ;
                }
      
                public boolean handleEvent(Event evt) {
                     if(evt.id == Event.ACTION_EVENT) {
                        this.handleCho() ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_ABSOLUTE) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_LINE_DOWN) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_LINE_UP) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_PAGE_DOWN) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_PAGE_UP) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     else return false ;
                }

                public void handleBar(Event evt) {
                   int i1,i2,i3,i4 ;
                   double v1,v2,v3,v4 ;
                   float fl1,fl2,fl3,fl4 ;
            // Input    for computations
                   i1 = s1.getValue() ;
                   i2 = s2.getValue() ;
                   i3 = s3.getValue() ;
                   i4 = s4.getValue() ;
   
                   if (ktype == 5) {
                     if (i3 > i2) {
                        i3 = i2 - 1 ;
                        s3.setValue(i3) ;
                     }
                     if (i4 > i2) {
                        i4 = i2 - 1 ;
                        s4.setValue(i3) ;
                     }
                   }

                   if (i3 > i4) {
                     i4 = i3 + 1 ;
                     s4.setValue(i4) ;
                   }

                   h1d = v1 = i1 * (hmxd - hmnd)/ 1000. + hmnd ;
                   h1 = h1d / lconv ;
                   h2d = v2 = i2 * (hmxd - hmnd)/ 1000. + hmnd ;
                   h2 = h2d / lconv ;
                   w1d = v3 = i3 * (wmxd - wmnd)/ 1000. + wmnd ;
                   if (ktype < 6) w1 = w1d / lconv ;
                   if (ktype == 6) w1 = w1d ;
                   w2d = v4 = i4 * (wmxd - wmnd)/ 1000. + wmnd ;
                   w2 = w2d / lconv ;
   
                   fl1 = (float) v1 ;
                   fl2 = (float) v2 ;
                   fl3 = (float) v3 ;
                   fl4 = (float) v4 ;
   
                   l.f1.setText(String.valueOf(fl1)) ;
                   l.f2.setText(String.valueOf(fl2)) ;
                   l.f3.setText(String.valueOf(fl3)) ;
                   l.f4.setText(String.valueOf(fl4)) ;
                   
                   compute() ;
                } // handle bar
 
                public void handleCho() {
                   int i1,i2 ;
                   double v1,v2 ;
                   double wdisplay ;
                   float fl1,fl2 ;
   
                   ktype = ktch.getSelectedIndex() ;

                   if (ktype == 0) {//default for diamond
                      h1 = .5 * w1 ;
                      h2 =  w1 ;
                      lbrid = 1.55 * w1 ;
                      lknot = 1.2 * w1 ;
                   }
                   if (ktype == 1) {//default for delta
                      h1 = .5 * w1 ;
                      h2 = .15 * w1 ;
                      lbrid = .85 * w1 ;
                      lknot = .5 * w1 ;
                   }
                   if (ktype == 2) {//default for sled
                      h1 = .5 * w1 ;
                      h2 =  w1 ;
                      w2 =  2.0 * w1 ;
                      lbrid = 1.5 * w1 ;
                      lknot = 1.5 * w1 ;
                   }
                   if (ktype == 3) {//default for box
                      h1 = .5 * w1 ;
                      h2 =  w1 ;
                      lbrid = 2.25 * w1 ;
                      lknot = 1.5 * w1 ;
                   }
                   if (ktype == 4) {//default for winged box
                      h1 = .5 * w1 ;
                      h2 =  w1 ;
                      w2 =  2.0 * w1 ;
                      lbrid = 2.25 * w1 ;
                      lknot = 1.5 * w1 ;
                   }
                   if (ktype == 5) {//defaults for twin trap
                      h1 = 8.5 ;
                      h2 = 11.0 ;
                      w1 = 2.75 ;
                      w2 = 7.0 ;

                      wtarea = .0012832 ;
                      wdisplay = wtarea * wconv / lconv / lconv ;
                      fl1 = (float) wdisplay ;
                      mat.f1.setText(String.valueOf(fl1)) ;
                      mat.matsur.select(3) ;

                      wtlngs = .00090 ;
                      wdisplay = wtlngs * wconv / lconv ;
                      fl1 = (float) wdisplay ;
                      mat.f2.setText(String.valueOf(fl1)) ;
                      mat.matstk.select(4) ;
                   }
                   if (ktype == 6) {//default for tumbleweed
                      h1 = 5.0 ;
                      h2 = w2 = h1 ;
                      w1 = 8.0 ;
                      lbrid = 0.0 ;
                      lknot = 0.0 ;
                   }
 
                   w1d = w1 * lconv ;
                   w2d = w2 * lconv ;
                   h1d = h1 * lconv ;
                   h2d = h2 * lconv ;
                   lbridd = lbrid * lconv ;
                   lknotd = lknot * lconv ;

                   fl1 = (float) h1d ;
                   i1 = (int) (((h1d - hmnd)/(hmxd-hmnd))*1000.) ;
                   l.f1.setText(String.valueOf(fl1)) ;
                   s1.setValue(i1) ;

                   fl1 = (float) h2d ;
                   i1 = (int) (((h2d - hmnd)/(hmxd-hmnd))*1000.) ;
                   l.f2.setText(String.valueOf(fl1)) ;
                   s2.setValue(i1) ;

                   fl1 = (float) w1d ;
                   i1 = (int) (((w1d - wmnd)/(wmxd-wmnd))*1000.) ;
                   l.f3.setText(String.valueOf(fl1)) ;
                   s3.setValue(i1) ;

                   fl1 = (float) w2d ;
                   i1 = (int) (((w2d - wmnd)/(wmxd-wmnd))*1000.) ;
                   l.f4.setText(String.valueOf(fl1)) ;
                   s4.setValue(i1) ;

                   fl1 = (float) lbridd ;
                   i1 = (int) (((lbridd - lbrmnd)/(lbrmxd-lbrmnd))*1000.) ;
                   trm.l.f1.setText(String.valueOf(fl1)) ;
                   trm.r.s1.setValue(i1) ;

                   fl1 = (float) lknotd ;
                   i1 = (int) (((lknotd - lbrmnd)/(lbrmxd-lbrmnd))*1000.) ;
                   trm.l.f2.setText(String.valueOf(fl1)) ;
                   trm.r.s2.setValue(i1) ;

                   compute() ;
                } // handleCho
              }  // Inright
           }  // Shape

           class Mat extends Panel {
              Kite outerparent ;
              Label l1,l2,l3,l4,l5 ;
              Choice matsur,matstk,matlin,matail;
              TextField f1,f2,f3,f4,f5 ;
              Button bt1,bt2,btc;
   
              Mat (Kite target) {
                 outerparent = target ;
                 setLayout(new GridLayout(6,3,5,5)) ;
    
                 bt1 = new Button("Shape") ;
                 bt1.setBackground(Color.white) ;
                 bt1.setForeground(Color.blue) ;
   
                 bt2 = new Button("Material") ;
                 bt2.setBackground(Color.yellow) ;
                 bt2.setForeground(Color.blue) ;

                 btc = new Button("Compute") ;
                 btc.setBackground(Color.white) ;
                 btc.setForeground(Color.red) ;

                 l1 = new Label("Surface-oz/sq in", Label.CENTER) ;
                 matsur = new Choice() ;
                 matsur.addItem("Plastic") ;
                 matsur.addItem("Tissue Paper");
                 matsur.addItem("Rip Stop");
                 matsur.addItem("Paper");
                 matsur.addItem("Silk Span");
                 matsur.addItem("Cellophane");
                 matsur.addItem("<-- Specify");
                 matsur.setBackground(Color.white) ;
                 matsur.setForeground(Color.red) ;
                 matsur.select(0) ;
                 f1 = new TextField(".0004752",5) ;
                 f1.setBackground(Color.black) ;
                 f1.setForeground(Color.green) ;

                 l2 = new Label("Frame-oz/in", Label.CENTER) ;
                 matstk = new Choice() ;
                 matstk.addItem("1/4 Balsa") ;
                 matstk.addItem("1/8 Balsa");
                 matstk.addItem("1/4 Birch");
                 matstk.addItem("3/8 Plastic Tube");
                 matstk.addItem("Skewer Stick");
                 matstk.addItem("<-- Specify");
                 matstk.setBackground(Color.white) ;
                 matstk.setForeground(Color.red) ;
                 matstk.select(0) ;
                 f2 = new TextField(".003952",5) ;
                 f2.setBackground(Color.black) ;
                 f2.setForeground(Color.green) ;

                 l3 = new Label("Line-oz/ft", Label.CENTER) ;
                 matlin = new Choice() ;
                 matlin.addItem("Nylon") ;
                 matlin.addItem("Cotton");
                 matlin.addItem("<-- Specify");
                 matlin.setBackground(Color.white) ;
                 matlin.setForeground(Color.red) ;
                 matlin.select(0) ;
                 f3 = new TextField(".004",5) ;
                 f3.setBackground(Color.black) ;
                 f3.setForeground(Color.green) ;

                 l4 = new Label("Tail-oz/in", Label.CENTER) ;
                 matail = new Choice() ;
                 matail.addItem("1 in Plastic") ;
                 matail.addItem("3 in Plastic") ;
                 matail.addItem("1 in Nylon");
                 matail.addItem("<-- Specify");
                 matail.setBackground(Color.white) ;
                 matail.setForeground(Color.red) ;
                 matail.select(0) ;
                 f4 = new TextField(".004752",5) ;
                 f4.setBackground(Color.black) ;
                 f4.setForeground(Color.green) ;

                 l5 = new Label("Max Dimension in", Label.CENTER) ;
                 f5 = new TextField("120.0",5) ;
                 f5.setBackground(Color.white) ;
                 f5.setForeground(Color.black) ;

                 add(bt1) ;
                 add(bt2) ;
                 add(new Label("Density", Label.LEFT)) ;

                 add(l1) ;
                 add(f1) ;
                 add(matsur) ;

                 add(l2) ;
                 add(f2) ;
                 add(matstk) ;

                 add(l4) ;
                 add(f4) ;
                 add(matail) ;

                 add(l3) ;
                 add(f3) ;
                 add(matlin) ;

                 add(l5) ;
                 add(f5) ;
                 add(btc) ;
              }

              public boolean action(Event evt, Object arg) {
                 if(evt.target instanceof Button) {
                    this.handleDispla(arg) ;
                    return true ;
                 }
                 if(evt.id == Event.ACTION_EVENT) {
                    this.handleText(evt) ;
                    return true ;
                 }
                 else return false ;
              }

              public void handleText(Event evt) {
                 Double V1,V2,V3,V4 ;
                 double v1,v2,v3,v4 ;
                 double wdisplay ;
                 float fl1 ;
  
                 surtyp = matsur.getSelectedIndex() ;
                 stktyp = matstk.getSelectedIndex() ;
                 lintyp = matlin.getSelectedIndex() ;
                 taltyp = matail.getSelectedIndex() ;

                 switch (surtyp) { // surface weight per area
                     case 0: {    // plastic 
                         wtarea = .0004752 ;
                         wdisplay = wtarea * wconv / lconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f1.setText(String.valueOf(fl1)) ;
                         f1.setBackground(Color.black) ;
                         f1.setForeground(Color.green) ;
                         break;
                     }
                     case 1: {   // tissue
                         wtarea = .0004576 ;
                         wdisplay = wtarea * wconv / lconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f1.setText(String.valueOf(fl1)) ;
                         f1.setBackground(Color.black) ;
                         f1.setForeground(Color.green) ;
                         break ;
                     }
                     case 2: {   // rip stop
                         wtarea = .0009552 ;
                         wdisplay = wtarea * wconv / lconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f1.setText(String.valueOf(fl1)) ;
                         f1.setBackground(Color.black) ;
                         f1.setForeground(Color.green) ;
                         break ;
                     }
                     case 3: {   // paper
                         wtarea = .0012832 ;
                         wdisplay = wtarea * wconv / lconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f1.setText(String.valueOf(fl1)) ;
                         f1.setBackground(Color.black) ;
                         f1.setForeground(Color.green) ;
                         break ;
                     }
                     case 4: {   // silk span
                         wtarea = .000384 ;
                         wdisplay = wtarea * wconv / lconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f1.setText(String.valueOf(fl1)) ;
                         f1.setBackground(Color.black) ;
                         f1.setForeground(Color.green) ;
                         break ;
                     }
                     case 5: {   // celophane
                         wtarea = .000656 ;
                         wdisplay = wtarea * wconv / lconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f1.setText(String.valueOf(fl1)) ;
                         f1.setBackground(Color.black) ;
                         f1.setForeground(Color.green) ;
                         break ;
                     }
                     case 6: {   // specify
                         V1 = Double.valueOf(f1.getText()) ;
                         wdisplay = V1.doubleValue() ;
                         wtarea = wdisplay * lconv * lconv / wconv ;
                         f1.setBackground(Color.white) ;
                         f1.setForeground(Color.black) ;
                         break ;
                     }
                 }

                 switch (stktyp) { // stick weight per length
                     case 0: {    // quarter balsa
                         wtlngs = .003952 ;
                         wdisplay = wtlngs * wconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f2.setText(String.valueOf(fl1)) ;
                         f2.setBackground(Color.black) ;
                         f2.setForeground(Color.green) ;
                         break;
                     }
                     case 1: {   // eighth balsa
                         wtlngs = .001968 ;
                         wdisplay = wtlngs * wconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f2.setText(String.valueOf(fl1)) ;
                         f2.setBackground(Color.black) ;
                         f2.setForeground(Color.green) ;
                         break ;
                     }
                     case 2: {   //  quarter birch
                         wtlngs = .0216 ;
                         wdisplay = wtlngs * wconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f2.setText(String.valueOf(fl1)) ;
                         f2.setBackground(Color.black) ;
                         f2.setForeground(Color.green) ;
                         break ;
                     }
                     case 3: {   // plastic tube
                         wtlngs = .02096 ;
                         wdisplay = wtlngs * wconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f2.setText(String.valueOf(fl1)) ;
                         f2.setBackground(Color.black) ;
                         f2.setForeground(Color.green) ;
                         break ;
                     }
                     case 4: {   // skewer stick
                         wtlngs = .00325 ;
                         wdisplay = wtlngs * wconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f2.setText(String.valueOf(fl1)) ;
                         f2.setBackground(Color.black) ;
                         f2.setForeground(Color.green) ;
                         break ;
                     }
                     case 5: {   // specify
                         V2 = Double.valueOf(f2.getText()) ;
                         wdisplay = V2.doubleValue() ;
                         wtlngs = wdisplay * lconv / wconv ;
                         f2.setBackground(Color.white) ;
                         f2.setForeground(Color.black) ;
                         break ;
                      }
                 }

                 switch (lintyp) { // line weight per foot
                     case 0: {    // nylon
                         wtlngl = .004 ;
                         wdisplay = wtlngl * wconv / hconv ;
                         fl1 = (float) wdisplay ;
                         f3.setText(String.valueOf(fl1)) ;
                         f3.setBackground(Color.black) ;
                         f3.setForeground(Color.green) ;
                         break;
                     }
                     case 1: {   // cotton
                         wtlngl = .002 ;
                         wdisplay = wtlngl * wconv / hconv ;
                         fl1 = (float) wdisplay ;
                         f3.setText(String.valueOf(fl1)) ;
                         f3.setBackground(Color.black) ;
                         f3.setForeground(Color.green) ;
                         break ;
                     }
                     case 2: {   // specify
                         V3 = Double.valueOf(f3.getText()) ;
                         wdisplay = V3.doubleValue() ;
                         wtlngl = wdisplay * hconv / wconv ;
                         f3.setBackground(Color.white) ;
                         f3.setForeground(Color.black) ;
                         break ;
                     }
                 }

                 switch (taltyp) { // tail weight per length
                     case 0: {    // 1 inch plastic
                         wttail = .0004752 ;
                         wdisplay = wttail * wconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f4.setText(String.valueOf(fl1)) ;
                         f4.setBackground(Color.black) ;
                         f4.setForeground(Color.green) ;
                         break;
                     }
                     case 1: {    // 3 inch plastic
                         wttail = .0014256 ;
                         wdisplay = wttail * wconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f4.setText(String.valueOf(fl1)) ;
                         f4.setBackground(Color.black) ;
                         f4.setForeground(Color.green) ;
                         break;
                     }
                     case 2: {   // 1 inch nylon
                         wttail = .000522 ;
                         wdisplay = wttail * wconv / lconv ;
                         fl1 = (float) wdisplay ;
                         f4.setText(String.valueOf(fl1)) ;
                         f4.setBackground(Color.black) ;
                         f4.setForeground(Color.green) ;
                         break ;
                     }
                     case 3: {   // specify
                         V4 = Double.valueOf(f4.getText()) ;
                         wdisplay = V4.doubleValue() ;
                         wttail = wdisplay * lconv / wconv ;
                         f4.setBackground(Color.white) ;
                         f4.setForeground(Color.black) ;
                         break ;
                     }
                 }

                 compute();
              } // TextHandler

              public void handleDispla(Object arg) {
                String label = (String)arg ;
                 Double V1,V2,V3,V4,V5 ;
                 double v1,v2,v3,v4,v5 ;
                 double wdisplay ;
                 float fl1 ;

                if (label.equals("Shape")) {
                   layin.show(in, "first")  ;
                }

                if (label.equals("Material")) {
                   layin.show(in, "second")  ;
                }
                if (label.equals("Compute")) {
                   layin.show(in, "second")  ;

                   V1 = Double.valueOf(f1.getText()) ;
                   wdisplay = V1.doubleValue() ;
                   wtarea = wdisplay * lconv * lconv / wconv ;

                   V2 = Double.valueOf(f2.getText()) ;
                   wdisplay = V2.doubleValue() ;
                   wtlngs = wdisplay * lconv / wconv ;

                   V3 = Double.valueOf(f3.getText()) ;
                   wdisplay = V3.doubleValue() ;
                   wtlngl = wdisplay * hconv / wconv ;

                   V4 = Double.valueOf(f4.getText()) ;
                   wdisplay = V4.doubleValue() ;
                   wttail = wdisplay * lconv / wconv ;

                   V5 = Double.valueOf(f5.getText()) ;
                   wdisplay = V5.doubleValue() ;
                   hmx = wdisplay / lconv ;
                   wmx = wdisplay / lconv ;
                   lbrmx = (2.0 * wdisplay + 10.0) / lconv ;

                   loadInput() ;
                   compute() ;
                }

              } // Display handler
           }  // Mate

           class Flt extends Panel {
              Kite outerparent ;
              L l ;
              R r ;
   
              Flt (Kite target) {
                 outerparent = target ;
                 setLayout(new GridLayout(1,2,5,5)) ;
   
                 l = new L(outerparent) ;
                 r = new R(outerparent) ;
   
                 add(l) ;
                 add(r) ;
              }
   
              class L extends Panel {
                 Kite outerparent ;
                 TextField f1,f2,f3,f4 ;
                 Label l1,l2,l3,l4 ;
                 TextField diag1,diag2,diag3,diag4 ;
   
                 L (Kite target) {
                  outerparent = target ;
                  setLayout(new GridLayout(5,2,2,10)) ;
 
                  l1 = new Label("Wind-ft/sec", Label.CENTER) ;
                  f1 = new TextField("20.0",5) ;
   
                  l2 = new Label("Altitude-ft", Label.CENTER) ;
                  f2 = new TextField("20.0",5) ;
   
                  l3 = new Label("Line - ft", Label.CENTER) ;
                  f3 = new TextField("100.0",5) ;
   
                  l4 = new Label("Payload-oz", Label.CENTER) ;
                  f4 = new TextField("0.0",5) ;
   
                  diag1 = new TextField("0.0",5) ;
                  diag2 = new TextField("0.0",5) ;
                  diag3 = new TextField("0.0",5) ;
                  diag4 = new TextField("0.0",5) ;

                  add(new Label(" ", Label.CENTER)) ;
                  add(new Label(" ", Label.CENTER)) ;

                  add(l1) ;
                  add(f1) ;
   
                  add(l2) ;
                  add(f2) ;

                  add(l3) ;
                  add(f3) ;
   
                  add(l4) ;
                  add(f4) ;
/*
                   add(diag1) ;
                   add(diag2) ;

                   add(diag3) ;
                   add(diag4) ;
*/                
                }
    
                public boolean action(Event evt, Object arg) {
                    if(evt.id == Event.ACTION_EVENT) {
                       this.handleText(evt) ;
                       return true ;
                    }
                    else return false ;
                }

                public void handleText(Event evt) {
                  Double V1,V2,V3,V4,V5 ;
                  double v1,v2,v3,v4,v5 ;
                  float fl1 ;
                  int i1,i2,i3,i4,i5 ;
   
                  V1 = Double.valueOf(f1.getText()) ;
                  v1 = V1.doubleValue() ;
                  V2 = Double.valueOf(f2.getText()) ;
                  v2 = V2.doubleValue() ;
                  V3 = Double.valueOf(f3.getText()) ;
                  v3 = V3.doubleValue() ;
                  V4 = Double.valueOf(f4.getText()) ;
                  v4 = V4.doubleValue() ;
   
                  windd = v1 ;
                  if(windd < wndmnd) {
                    windd = v1 = wndmnd ;
                    fl1 = (float) v1 ;
                    f1.setText(String.valueOf(fl1)) ;
                  }
                  if(windd > wndmxd) {
                    windd = v1 = wndmxd ;
                    fl1 = (float) v1 ;
                    f1.setText(String.valueOf(fl1)) ;
                  }
                  wind = windd / hconv ;
   
                  altd = v2 ;
                  if(altd < altmnd) {
                    altd = v2 = altmnd ;
                    fl1 = (float) v2 ;
                    f2.setText(String.valueOf(fl1)) ;
                  }
                  if(altd > altmxd) {
                    altd = v2 = altmxd ;
                    fl1 = (float) v2 ;
                    f2.setText(String.valueOf(fl1)) ;
                  }
                  alt = altd / hconv ;
   
                  llined = v3 ;
                  if(llined < llnmnd) {
                    llined = v3 = llnmnd ;
                    fl1 = (float) v3 ;
                    f3.setText(String.valueOf(fl1)) ;
                  }
                  if(llined > llnmxd) {
                    llined = v3 = llnmxd ;
                    fl1 = (float) v3 ;
                    f3.setText(String.valueOf(fl1)) ;
                  }
                  lline = llined / hconv ;
   
                  wpayd = v4 ;
                  if(wpayd < wpaymnd) {
                    wpayd = v4 = wpaymnd ;
                    fl1 = (float) v4 ;
                    f4.setText(String.valueOf(fl1)) ;
                  }
                  if(wpayd > wpaymxd) {
                    wpayd = v4 = wpaymxd ;
                    fl1 = (float) v4 ;
                    f4.setText(String.valueOf(fl1)) ;
                  }
                  wpay = wpayd / wconv ;
   
                  i1 = (int) (((v1 - wndmnd)/(wndmxd-wndmnd))*1000.) ;
                  i2 = (int) (((v2 - altmnd)/(altmxd-altmnd))*1000.) ;
                  i3 = (int) (((v3 - llnmnd)/(llnmxd-llnmnd))*1000.) ;
                  i4 = (int) (((v4 - wpaymnd)/(wpaymxd-wpaymnd))*1000.) ;
   
                  r.s1.setValue(i1) ;
                  r.s2.setValue(i2) ;
                  r.s3.setValue(i3) ;
                  r.s4.setValue(i4) ;

                  compute();
                } // TextHandler
              }  // Inleft
 
              class R extends Panel {
                 Kite outerparent ;
                 Scrollbar s1,s2,s3,s4;
                 Choice plntch;
   
                 R (Kite target) {
                  int i1,i2,i3,i4 ;
   
                  outerparent = target ;
                  setLayout(new GridLayout(5,1,2,10)) ;
   
                  i1 = (int) (((20.0 - wndmn)/(wndmx-wndmn))*1000.) ;
                  i2 = (int) (((0.0 - altmn)/(altmx-altmn))*1000.) ;
                  i3 = (int) (((100.0 - llnmn)/(llnmx-llnmn))*1000.) ;
                  i4 = (int) (((0.0 - wpaymn)/(wpaymx-wpaymn))*1000.) ;
   
                  s1 = new Scrollbar(Scrollbar.HORIZONTAL,i1,10,0,1000);
                  s2 = new Scrollbar(Scrollbar.HORIZONTAL,i2,10,0,1000);
                  s3 = new Scrollbar(Scrollbar.HORIZONTAL,i3,10,0,1000);
                  s4 = new Scrollbar(Scrollbar.HORIZONTAL,i4,10,0,1000);
 
                  plntch = new Choice() ;
                  plntch.addItem("Earth - Average Day") ;
                  plntch.addItem("Mars - Average Day");
                  plntch.setBackground(Color.white) ;
                  plntch.setForeground(Color.blue) ;
                  plntch.select(0) ;

                  add(plntch) ;
                  add(s1) ;
                  add(s2) ;
                  add(s3) ;
                  add(s4) ;
                }
      
                public boolean handleEvent(Event evt) {
                     if(evt.id == Event.ACTION_EVENT) {
                        this.handleCho(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_ABSOLUTE) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_LINE_DOWN) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_LINE_UP) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_PAGE_DOWN) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_PAGE_UP) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     else return false ;
                }

                public void handleBar(Event evt) {
                   int i1,i2,i3,i4 ;
                   double v1,v2,v3,v4 ;
                   float fl1,fl2,fl3,fl4 ;
            // Input    for computations
                   i1 = s1.getValue() ;
                   i2 = s2.getValue() ;
                   i3 = s3.getValue() ;
                   i4 = s4.getValue() ;
   
                   windd = v1 = i1 * (wndmxd - wndmnd)/ 1000. + wndmnd ;
                   wind = windd / hconv ;
                   altd = v2 = i2 * (altmxd - altmnd)/ 1000. + altmnd ;
                   alt = altd / hconv ;
                   llined = v3 = i3 * (llnmxd - llnmnd)/ 1000. + llnmnd ;
                   lline = llined / hconv;
                   wpayd = v4 = i4 * (wpaymxd - wpaymnd)/ 1000. + wpaymnd ;
                   wpay = wpayd / wconv;
 
                   fl1 = (float) v1 ;
                   fl2 = (float) v2 ;
                   fl3 = (float) v3 ;
                   fl4 = (float) v4 ;
   
                   l.f1.setText(String.valueOf(fl1)) ;
                   l.f2.setText(String.valueOf(fl2)) ;
                   l.f3.setText(String.valueOf(fl3)) ;
                   l.f4.setText(String.valueOf(fl4)) ;
                   
                   compute() ;
                } // handle bar
 
                public void handleCho(Event evt) {

                   planet  = plntch.getSelectedIndex() ;
                   compute() ;

                } // end choice
              }  // Inright
           } // Flight

           class Trm extends Panel {
              Kite outerparent ;
              L l ;
              R r ;
   
              Trm (Kite target) {
                 outerparent = target ;
                 setLayout(new GridLayout(1,2,5,5)) ;
   
                 l = new L(outerparent) ;
                 r = new R(outerparent) ;
   
                 add(l) ;
                 add(r) ;
              }
   
              class L extends Panel {
                 Kite outerparent ;
                 TextField f1,f2,f3,f4 ;
                 Label l1,l2,l3,l4 ;
   
                 L (Kite target) {
                  outerparent = target ;
                  setLayout(new GridLayout(5,2,2,10)) ;
 
                  l1 = new Label("B-Bridle-in", Label.CENTER) ;
                  f1 = new TextField("17.0",5) ;
   
                  l2 = new Label("K-Knot-in", Label.CENTER) ;
                  f2 = new TextField("12.5",5) ;
   
                  l3 = new Label("Angle", Label.CENTER) ;
                  f3 = new TextField("5.0",5) ;
   
                  l4 = new Label("T-Tail-in", Label.CENTER) ;
                  f4 = new TextField("6.0",5) ;
   
                  add(new Label(" ", Label.CENTER)) ;
                  add(new Label(" ", Label.CENTER)) ;

                  add(l3) ;
                  add(f3) ;

                  add(l1) ;
                  add(f1) ;
   
                  add(l2) ;
                  add(f2) ;

                  add(l4) ;
                  add(f4) ;

                }
    
                public boolean action(Event evt, Object arg) {
                    if(evt.id == Event.ACTION_EVENT) {
                       this.handleText(evt) ;
                       return true ;
                    }
                    else return false ;
                }

                public void handleText(Event evt) {
                  Double V1,V2,V3,V4 ;
                  double v1,v2,v3,v4 ;
                  float fl1 ;
                  int i1,i2,i3,i4;
   
                  V1 = Double.valueOf(f1.getText()) ;
                  v1 = V1.doubleValue() ;
                  V2 = Double.valueOf(f2.getText()) ;
                  v2 = V2.doubleValue() ;
                  V4 = Double.valueOf(f4.getText()) ;
                  v4 = V4.doubleValue() ;
   
                  if (ktype < 5) {
                    lbridd = v1 ;
                    if (lbridd < lkite*lconv) {
                      lbridd = v1 = lkite*lconv + .01 ;
                      fl1 = (float) v1 ;
                      f1.setText(String.valueOf(fl1)) ;
                    }
                    if(lbridd < lbrmnd) {
                      lbridd = v1 = lbrmnd ;
                      fl1 = (float) v1 ;
                      f1.setText(String.valueOf(fl1)) ;
                    }
                    if(lbridd > lbrmxd) {
                      lbridd = v1 = lbrmxd ;
                      fl1 = (float) v1 ;
                      f1.setText(String.valueOf(fl1)) ;
                    }
                    lbrid = lbridd / lconv ;
                  }
                  if (ktype == 6 ) {
                    lbridd = v1 ;
                    if(lbridd < lbrmnd) {
                      lbridd = v1 = lbrmnd ;
                      fl1 = (float) v1 ;
                      f1.setText(String.valueOf(fl1)) ;
                    }
                    if(lbridd > lbrmxd) {
                      lbridd = v1 = lbrmxd ;
                      fl1 = (float) v1 ;
                      f1.setText(String.valueOf(fl1)) ;
                    }
                  }
 
                  lknotd = v2 ;
                  if (ktype <= 4) {
                     if(lknotd > lbridd) {
                       lknotd = v2 = lbridd ;
                       fl1 = (float) v2 ;
                       f2.setText(String.valueOf(fl1)) ;
                     }
                  }
                  if (ktype == 5) {
                     if(lknotd > lkite*lconv) {
                       lknotd = v2 = lkite*lconv - .1 ;
                       fl1 = (float) v2 ;
                       f2.setText(String.valueOf(fl1)) ;
                     }
                  }
                  if(lknotd < lbrmnd) {
                    lknotd = v2 = lbrmnd ;
                    fl1 = (float) v2 ;
                    f2.setText(String.valueOf(fl1)) ;
                  }
                  if(lknotd > lbrmxd) {
                    lknotd = v2 = lbrmxd ;
                    fl1 = (float) v2 ;
                    f2.setText(String.valueOf(fl1)) ;
                  }
                  lknot = lknotd / lconv ;
 
                  ltaild = v4 ;
                  if(ltaild < ltlmnd) {
                    ltaild = v4 = ltlmnd ;
                    fl1 = (float) v4 ;
                    f4.setText(String.valueOf(fl1)) ;
                  }
                  if(ltaild > ltlmxd) {
                    ltaild = v4 = ltlmxd ;
                    fl1 = (float) v4 ;
                    f4.setText(String.valueOf(fl1)) ;
                  }
                  ltail = ltaild / lconv ;
   
                  i1 = (int) (((v1 - lbrmnd)/(lbrmxd-lbrmnd))*1000.) ;
                  i2 = (int) (((v2 - lbrmnd)/(lbrmxd-lbrmnd))*1000.) ;
                  i4 = (int) (((v4 - ltlmnd)/(ltlmxd-ltlmnd))*1000.) ;
   
                  r.s1.setValue(i1) ;
                  r.s2.setValue(i2) ;
                  r.s4.setValue(i4) ;

                  if (angmod > 0) {
                    V3 = Double.valueOf(f3.getText()) ;
                    v3 = V3.doubleValue() ;
                    alpha = v3 ;
                    if(alpha < almn) {
                      alpha = v3 = almn ;
                      fl1 = (float) v3 ;
                      f3.setText(String.valueOf(fl1)) ;
                    }
                    if(alpha > almx) {
                      alpha = v3 = almx ;
                      fl1 = (float) v3 ;
                      f3.setText(String.valueOf(fl1)) ;
                    }
     
                    i3 = (int) (((v3 - almn)/(almx-almn))*1000.) ;
                    r.s3.setValue(i3) ;
                  }
   
                  compute();
                } // TextHandler
              }  // Inleft
 
              class R extends Panel {
                 Kite outerparent ;
                 Scrollbar s1,s2,s3,s4;
                 Choice amode;
   
                 R (Kite target) {
                  int i1,i2,i3,i4;
   
                  outerparent = target ;
                  setLayout(new GridLayout(5,1,2,10)) ;
   
                  i1 = (int) (((20.0 - lbrmn)/(lbrmx-lbrmn))*1000.) ;
                  i2 = (int) (((10.0 - lbrmn)/(lbrmx-lbrmn))*1000.) ;
                  i3 = (int) (((0.0 - almn)/(almx-almn))*1000.) ;
                  i4 = (int) (((6.0 - ltlmn)/(ltlmx-ltlmn))*1000.) ;

                  amode = new Choice() ;
                  amode.addItem("Compute Trim Angle") ;
                  amode.addItem("Set Trim Angle ");
                  amode.setBackground(Color.white) ;
                  amode.setForeground(Color.red) ;
                  amode.select(0) ;

                  s1 = new Scrollbar(Scrollbar.HORIZONTAL,i1,10,0,1000);
                  s2 = new Scrollbar(Scrollbar.HORIZONTAL,i2,10,0,1000);
                  s3 = new Scrollbar(Scrollbar.HORIZONTAL,i3,10,0,1000);
                  s4 = new Scrollbar(Scrollbar.HORIZONTAL,i4,10,0,1000);
   
                  add(amode) ;
                  add(s3) ;
                  add(s1) ;
                  add(s2) ;
                  add(s4) ;
                }
      
                public boolean handleEvent(Event evt) {
                     if(evt.id == Event.ACTION_EVENT) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_ABSOLUTE) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_LINE_DOWN) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_LINE_UP) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_PAGE_DOWN) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     if(evt.id == Event.SCROLL_PAGE_UP) {
                        this.handleBar(evt) ;
                        return true ;
                     }
                     else return false ;
                }

                public void handleBar(Event evt) {
                   int i1,i2,i3,i4 ;
                   double v1,v2,v3,v4 ;
                   float fl1,fl2,fl3,fl4 ;
            // Input    for computations
                   i1 = s1.getValue() ;
                   i2 = s2.getValue() ;
                   i4 = s4.getValue() ;
                   angmod = amode.getSelectedIndex() ;

                   lbridd = v1 = i1 * (lbrmxd - lbrmnd)/ 1000. + lbrmnd ;
                   if (ktype < 6) {
                      lbrid = lbridd / lconv ;
                   }
                   if (ktype == 6) {
                      lbrid = lbridd ;
                   }
                   lknotd = v2 = i2 * (lbrmxd - lbrmnd)/ 1000. + lbrmnd ;
                   lknot = lknotd / lconv ;
                   ltaild = v4 = i4 * (ltlmxd - ltlmnd)/ 1000. + ltlmnd ;
                   ltail = ltaild / lconv ;
 
                   if (ktype <= 4) {
                      if (lbridd < lkite*lconv) {
                        lbridd = v1 = lkite*lconv + .01 ;
                        i1 = (int) (((lbridd - lbrmnd)/(lbrmxd-lbrmnd))*1000.) ;
                        s1.setValue(i1) ;
                      }
                      if (lknotd > lbridd) {
                        lknotd = v2 = lbridd ;
                        i2 = (int) (((lknotd - lbrmnd)/(lbrmxd-lbrmnd))*1000.) ;
                        s2.setValue(i2) ;
                      }
                   }
                   if (ktype == 5) {
                      if (lknotd > lkite*lconv) {
                        lknotd = v2 = lkite*lconv - .1 ;
                        i2 = (int) (((lknotd - lbrmnd)/(lbrmxd-lbrmnd))*1000.) ;
                        s2.setValue(i2) ;
                      }
                   }
   
                   fl1 = (float) v1 ;
                   fl2 = (float) v2 ;
                   fl4 = (float) v4 ;
   
                   l.f1.setText(String.valueOf(fl1)) ;
                   l.f2.setText(String.valueOf(fl2)) ;
                   l.f4.setText(String.valueOf(fl4)) ;
                   
                   if (angmod > 0) {
                      i3 = s3.getValue() ;
                      alpha = v3 = i3 * (almx - almn)/ 1000. + almn ;
                      fl3 = (float) v3 ;
                      l.f3.setText(String.valueOf(fl3)) ;
                   }

                   compute() ;
                } // handle bar
              }  // Inright
           } // Trim

           class Prt extends Panel {
              Kite outerparent ;
              TextArea prnt ;
   
              Prt (Kite target) {
                 outerparent = target ;
                 setLayout(new GridLayout(1,1,0,0)) ;
   
                 prnt = new TextArea() ;
                 prnt.setEditable(false) ;

                 prnt.appendText("KiteModeler 1.5a beta-28 Aug 12 --NASA Glenn\n") ;
                 add(prnt) ;
              }
           } // end Prt
        }  // Inppnl 

        class Out extends Panel {
           Kite outerparent ;
           Up up ;
           Dn dn ;
   
           Out (Kite target) { 
             outerparent = target ;
             setLayout(new GridLayout(2,1,2,2)) ;
    
             up = new Up(outerparent) ;
             dn = new Dn(outerparent) ;
   
             add(up) ;
             add(dn) ;
           }

           class Up extends Panel {
              Kite outerparent ;
              Label lo3,lo7,lo8,lo17 ;
              TextField o3,o7,o8,o17 ;
              Button bt3,bt4,bt5,bt6 ;
    
              Up (Kite target) { 
                outerparent = target ;
                setLayout(new GridLayout(3,4,2,2)) ;
    
                bt3 = new Button("Design") ;
                bt3.setBackground(Color.yellow) ;
                bt3.setForeground(Color.blue) ;
   
                bt4 = new Button("Trim") ;
                bt4.setBackground(Color.white) ;
                bt4.setForeground(Color.blue) ;
   
                bt5 = new Button("Fly") ;
                bt5.setBackground(Color.white) ;
                bt5.setForeground(Color.blue) ;
   
                bt6 = new Button("Output") ;
                bt6.setBackground(Color.white) ;
                bt6.setForeground(Color.blue) ;
   
                lo3 = new Label("Weight -oz", Label.CENTER) ;
                o3 = new TextField("25.0",5) ;
                o3.setBackground(Color.black) ;
                o3.setForeground(Color.green) ;

                lo7 = new Label("Lift -oz", Label.CENTER) ;
                o7 = new TextField("25.0",5) ;
                o7.setBackground(Color.black) ;
                o7.setForeground(Color.green) ;
   
                lo8 = new Label("Drag -oz", Label.CENTER) ;
                o8 = new TextField("25.0",5) ;
                o8.setBackground(Color.black) ;
                o8.setForeground(Color.green) ;

                lo17 = new Label("Tension-oz", Label.CENTER) ;
                o17 = new TextField("25.0",5) ;
                o17.setBackground(Color.black) ;
                o17.setForeground(Color.green) ;
   
                add(bt3) ;
                add(bt4) ;
                add(bt5) ;
                add(bt6) ;

                add(lo7) ;
                add(o7) ;
                add(lo8) ;
                add(o8) ;

                add(lo3) ;
                add(o3) ;
                add(lo17) ;
                add(o17) ;
             }

             public boolean action(Event evt, Object arg) {
               String label = (String)arg ;
   
               if(evt.target instanceof Button) {
                  if (label.equals("Design")) {
                     bt3.setBackground(Color.yellow) ;
                     bt4.setBackground(Color.white) ;
                     bt5.setBackground(Color.white) ;
                     bt6.setBackground(Color.white) ;
                     layin.show(in, "first")  ;
                     layout.show(dn, "fsto")  ;
                     if (pick == -1) restFrontView() ;
                  }
                  if (label.equals("Trim")) {
                     bt3.setBackground(Color.white) ;
                     bt4.setBackground(Color.yellow) ;
                     bt5.setBackground(Color.white) ;
                     bt6.setBackground(Color.white) ;
                     layin.show(in, "fourth")  ;
                     layout.show(dn, "fsto")  ;
                     if (pick == -1) restSideView() ;
                  }
                  if (label.equals("Fly")) {
                     bt3.setBackground(Color.white) ;
                     bt4.setBackground(Color.white) ;
                     bt5.setBackground(Color.yellow) ;
                     bt6.setBackground(Color.white) ;
                     layin.show(in, "third")  ;
                     layout.show(dn, "seco")  ;
                     if (pick == -1) restFieldView() ;
                  }
                  if (label.equals("Output")) {
                     bt3.setBackground(Color.white) ;
                     bt4.setBackground(Color.white) ;
                     bt5.setBackground(Color.white) ;
                     bt6.setBackground(Color.yellow) ;
                     layin.show(in, "fifth")  ;
                     layout.show(dn, "seco")  ;
                     if (pick == -1)  restFieldView() ;
                  }
                  loadInput() ;
                  compute() ;
                  return true ;
               }
               else return false ;
             } // Handler
           }  // end Up

           class Dn extends Panel {
              Kite outerparent ;
              Des des;
              Flt flt;
    
              Dn (Kite target) { 
                outerparent = target ;
                layout = new CardLayout() ;
                setLayout(layout) ;
    
                des = new Des(outerparent) ;
                flt = new Flt(outerparent) ;

                add ("fsto", des) ;
                add ("seco", flt) ;
              }

              class Des extends Panel {
                 Kite outerparent ;
                 Label lo1,lo2,lo4,lo5,lo9,lo10 ;
                 TextField o1,o2,o4,o5,o9,o10 ;

                 Des (Kite target) {
                   outerparent = target ;
                   setLayout(new GridLayout(3,4,2,2)) ;
    
                   lo1 = new Label("Surface-sqin", Label.CENTER) ;
                   o1 = new TextField("75.0",5) ;
                   o1.setBackground(Color.black) ;
                   o1.setForeground(Color.green) ;

                   lo2 = new Label("Frame-in", Label.CENTER) ;
                   o2 = new TextField("25.0",5) ;
                   o2.setBackground(Color.black) ;
                   o2.setForeground(Color.green) ;

                   lo4 = new Label("Cg -in", Label.CENTER) ;
                   o4 = new TextField("25.0",5) ;
                   o4.setBackground(Color.black) ;
                   o4.setForeground(Color.green) ;
   
                   lo5 = new Label("Cp -in", Label.CENTER) ;
                   o5 = new TextField("25.0",5) ;
                   o5.setBackground(Color.black) ;
                   o5.setForeground(Color.green) ;
      
                   lo9 = new Label("Torque", Label.CENTER) ;
                   o9 = new TextField("25.0",5) ;
                   o9.setBackground(Color.black) ;
                   o9.setForeground(Color.green) ;

                   lo10 = new Label("Angle", Label.CENTER) ;
                   o10 = new TextField("5.0",5) ;
                   o10.setBackground(Color.black) ;
                   o10.setForeground(Color.green) ;

                   add(lo4) ;
                   add(o4) ;
                   add(lo5) ;
                   add(o5) ;

                   add(lo1) ;
                   add(o1) ;
                   add(lo2) ;
                   add(o2) ;

                   add(lo9) ;
                   add(o9) ;
                   add(lo10) ;
                   add(o10) ;
                }
              }  //  end Des

              class Flt extends Panel {
                 Kite outerparent ;
                 Label lo12,lo13,lo14,lo15,lo9,lo10 ;
                 TextField o12,o13,o14,o15,o9,o10 ;

                 Flt (Kite target) {
                   outerparent = target ;
                   setLayout(new GridLayout(3,4,2,2)) ;
    
                   lo12 = new Label("Range-X-ft", Label.CENTER) ;
                   o12 = new TextField("75.0",5) ;
                   o12.setBackground(Color.black) ;
                   o12.setForeground(Color.green) ;

                   lo13 = new Label("Height-Y-ft", Label.CENTER) ;
                   o13 = new TextField("75.0",5) ;
                   o13.setBackground(Color.black) ;
                   o13.setForeground(Color.green) ;

                   lo14 = new Label("Pres psi", Label.CENTER) ;
                   o14 = new TextField("10.0",5) ;
                   o14.setBackground(Color.black) ;
                   o14.setForeground(Color.green) ;

                   lo15 = new Label("Temp F", Label.CENTER) ;
                   o15 = new TextField("25.0",5) ;
                   o15.setBackground(Color.black) ;
                   o15.setForeground(Color.green) ;
   
                   lo9 = new Label("Torque", Label.CENTER) ;
                   o9 = new TextField("25.0",5) ;
                   o9.setBackground(Color.black) ;
                   o9.setForeground(Color.green) ;

                   lo10 = new Label("Angle", Label.CENTER) ;
                   o10 = new TextField("5.0",5) ;
                   o10.setBackground(Color.black) ;
                   o10.setForeground(Color.green) ;

                   add(lo12) ;
                   add(o12) ;
                   add(lo13) ;
                   add(o13) ;

                   add(lo14) ;
                   add(o14) ;
                   add(lo15) ;
                   add(o15) ;

                   add(lo9) ;
                   add(o9) ;
                   add(lo10) ;
                   add(o10) ;
                }
              }  //  end Flt
           }  //  end Dn
        }  // Outpnl 
     } // Conppnl

     class Viewer extends Canvas  
            implements Runnable{
        Kite outerparent ;
        Thread runner ;
        Point locate,anchor;
   
        Viewer (Kite target) {
            setBackground(Color.blue) ;
            runner = null ;
        } 

        public Insets insets() {
           return new Insets(0,10,0,10) ;
        }
 
        public boolean mouseDown(Event evt, int x, int y) {
           anchor = new Point(x,y) ;
           return true;
        }

        public boolean mouseUp(Event evt, int x, int y) {
           handleb(x,y) ;
           return true;
        }

        public boolean mouseDrag(Event evt, int x, int y) {
           handle(x,y) ;
           return true;
        }

        public void handle(int x, int y) {  // slider widgets
         // determine location
           if (y > 15 && y < 370) {
             if (x <= 30 ) {   // zoom widget
               if (y >= 45 && y <= 275) {
                 sldloc = y ;
                 if (sldloc < 55) sldloc = 55;
                 if (sldloc > 275) sldloc = 275;
                 fact = 20.0 * 10.0/(sldloc-50);
               }
             }
             if (x >= 31 ) {   // translate
               locate = new Point(x,y) ;
               yt =  yt + (int) (.2*(locate.y - anchor.y)) ;
               xt =  xt + (int) (.4*(locate.x - anchor.x))  ;
               if (xt > 1000) xt = 1000 ;
               if (xt < -1000) xt = -1000 ;
               if (yt > 2000) yt = 2000 ;
               if (yt <-1000) yt = -1000 ;
             }
           }
           if (y >= 370 ) {   // scaling widgets
             if (x >= 95 && x <= 225) {
               if (viewflg >= 1) {  // force scale
                 fldloc = x ;
                 if (fldloc < 95) fldloc = 95;
                 if (fldloc > 225) fldloc = 225;
                 fscale = fscmin + (fldloc-95)*fscrat ;
               }
             }
           }
           if (viewflg == 2) {
             fact2 = fact ;
             xt2 = xt ;
             yt2 = yt ;
             sld2 = sldloc ;
             fsc2 = fscale ;
             fld2 = fldloc ;
           }
           if (viewflg == 1) {
             fact1 = fact ;
             xt1 = xt ;
             yt1 = yt ;
             sld1 = sldloc ;
             fsc1 = fscale ;
             fld1 = fldloc ;
           }
           if (viewflg == 0) {
             fact0 = fact ;
             xt0 = xt ;
             yt0 = yt ;
             sld0 = sldloc ;
           }
        }

        public void handleb(int x, int y) { // view buttons
          float fl3 ;
          int i3 ;

          if (y <= 15) {
             if (x >= 5 && x <= 60) {   // select units
                  units = -units ;
                  setUnits() ;
             }
             if (x >= 65 && x <= 140) {   // select view
                  pick = -pick ;
             }
             if (x >= 150 && x <= 199) {   // front view
                  if (pick == 1) restFrontView() ;
             }
             if (x >= 200 && x <= 240) {   // side view
                  if (pick == 1) restSideView() ;
             }
             if (x >= 241 && x <= 289 ) {   // field view
                  if (pick == 1) restFieldView() ;
             }
          }
          if (x >= 2 && x <= 31) {  // save button
             if (y >= 330 && y <= 350) { // print button
                printData() ;
             } 
             if (y >= 367 && y <= 387) {
                if (ktype <= 4) {  // Reset
                  w1 = 10.0 ;
                  w1d = w1*lconv ;

                  fl3 = (float) w1d ;
                  i3 = (int) (((w1d - wmnd)/(wmxd-wmnd))*1000.) ;

                  c.in.shp.l.f3.setText(String.valueOf(fl3)) ;
                  c.in.shp.r.s3.setValue(i3) ;
                  c.in.shp.r.handleCho() ;
                }
                if (ktype == 5 && viewflg == 0) {  // fold kite
                  fold = - fold ;
                }
             } 
             if (y >=31 && y <= 45) {  // find
               if (viewflg == 0) setFrontView() ;
               if (viewflg == 1) setSideView() ;
               if (viewflg == 2) setFieldView() ;
             }
          }

          view.repaint() ;
        }

        public void setUnits() { // units switch
    // change labels
           if (units == -1) {  // metric units
              lconv = 2.54 ;
              wconv = 28.35 ;
              hconv = .3048 ;
              c.out.up.lo3.setText("Weight gm") ;
              c.out.up.lo7.setText("Lift gm") ;
              c.out.up.lo8.setText("Drag gm") ;
              c.out.up.lo17.setText("Tension gm") ;
              c.out.dn.des.lo4.setText("Cg cm") ;
              c.out.dn.des.lo5.setText("Cp cm") ;
              c.out.dn.des.lo1.setText("Surface cm2") ;
              c.out.dn.des.lo2.setText("Frame cm") ;
              c.out.dn.flt.lo14.setText("Pres kPa") ;
              c.out.dn.flt.lo15.setText("Temp C") ;
              c.out.dn.flt.lo13.setText("Height-Y-m") ;
              c.out.dn.flt.lo12.setText("Range-X-m") ;
              c.in.shp.l.l1.setText("H1  cm") ;
              c.in.shp.l.l2.setText("H2  cm") ;
              c.in.shp.l.l3.setText("W1  cm") ;
              c.in.shp.l.l4.setText("W2  cm") ;
              c.in.trm.l.l1.setText("B-Bridle cm") ;
              c.in.trm.l.l2.setText("K-Knot cm") ;
              c.in.trm.l.l4.setText("T-Tail cm") ;
              c.in.flt.l.l4.setText("Payload gm") ;
              c.in.flt.l.l3.setText("Line  m") ;
              c.in.flt.l.l2.setText("Altitude m") ;
              c.in.flt.l.l1.setText("Wind m/sec") ;
              c.in.mat.l1.setText("Surface gm/cm2") ;
              c.in.mat.l2.setText("Frame gm/cm") ;
              c.in.mat.l4.setText("Tail gm/cm") ;
              c.in.mat.l3.setText("Line gm/m") ;
              c.in.mat.l5.setText("Max Dimension cm") ;
           }
           if (units == 1) {  // english units
              lconv = 1.0 ;
              wconv = 1.0 ;
              hconv = 1.0 ;
              c.out.up.lo3.setText("Weight oz") ;
              c.out.up.lo7.setText("Lift oz") ;
              c.out.up.lo8.setText("Drag oz") ;
              c.out.up.lo17.setText("Tension oz") ;
              c.out.dn.des.lo4.setText("Cg in") ;
              c.out.dn.des.lo5.setText("Cp in") ;
              c.out.dn.des.lo1.setText("Surface in2") ;
              c.out.dn.des.lo2.setText("Frame in") ;
              c.out.dn.flt.lo14.setText("Pres psi") ;
              c.out.dn.flt.lo15.setText("Temp F") ;
              c.out.dn.flt.lo13.setText("Height-Y-ft") ;
              c.out.dn.flt.lo12.setText("Range-X-ft") ;
              c.in.shp.l.l1.setText("H1  in") ;
              c.in.shp.l.l2.setText("H2  in") ;
              c.in.shp.l.l3.setText("W1  in") ;
              c.in.shp.l.l4.setText("W2  in") ;
              c.in.trm.l.l1.setText("B-Bridle in") ;
              c.in.trm.l.l2.setText("K-Knot in") ;
              c.in.trm.l.l4.setText("T-Tail in") ;
              c.in.flt.l.l4.setText("Payload oz") ;
              c.in.flt.l.l3.setText("Line  ft") ;
              c.in.flt.l.l2.setText("Altitude ft") ;
              c.in.flt.l.l1.setText("Wind ft/sec") ;
              c.in.mat.l1.setText("Surface oz/in2") ;
              c.in.mat.l2.setText("Frame oz/in") ;
              c.in.mat.l4.setText("Tail oz/in") ;
              c.in.mat.l3.setText("Line oz/ft") ;
              c.in.mat.l5.setText("Max Dimension in") ;
           }
           loadInput() ;
           compute() ;
        }

        public void start() {
           if (runner == null) {
              runner = new Thread(this) ;
              runner.start() ;
           }
        }

        public void run() {
          int timer ;
    
          timer = 100 ;
          while (true) {
             try { Thread.sleep(timer); }
             catch (InterruptedException e) {}
             view.repaint() ;
          }
        }

        public void update(Graphics g) {
           view.paint(g) ;
        }
 
        public void paint(Graphics g) {
           int i,j,k,n ;
           int exes[] = new int[9] ;
           int whys[] = new int[9] ;
           int ylabel,ylabel2,xlabel,xlabel2;
           double ripple,cang,sang,lrad,bang;
           Color col ;

           col = new Color(0,0,1) ;
 // FRONT VIEW
           if (viewflg == 0) {
              off1Gg.setColor(Color.blue) ;
              off1Gg.fillRect(0,0,500,500) ;
              if (ktype == 0) {  // diamond
                 off1Gg.setColor(Color.white) ;
                 exes[0] = (int) (fact*(0.0)) + xt ;
                 whys[0] = (int) (fact*(-h1 - h2)) + yt ;
                 exes[1] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[1] = (int) (fact*(-h2)) + yt ;
                 exes[2] = (int) (fact*(0.0)) + xt ;
                 whys[2] = (int) (fact*(0.0)) + yt ;
                 exes[3] = (int) (fact*(w1/2.0)) + xt ;
                 whys[3] = (int) (fact*(-h2)) + yt ;
                 exes[4] = (int) (fact*(0.0)) + xt ;
                 whys[4] = (int) (fact*(-h1 - h2)) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;
                 off1Gg.setColor(Color.black) ;
                 off1Gg.drawLine(exes[0],whys[0],exes[2],whys[2]) ;
                 off1Gg.drawLine(exes[1],whys[1],exes[3],whys[3]) ;
               //Labels
                 off1Gg.setColor(Color.yellow) ;
                 off1Gg.drawString("W1",xt-5,337) ;
                 off1Gg.drawLine(exes[1],327,exes[1],337) ;
                 off1Gg.drawLine(exes[1],332,xt-20,332) ;
                 off1Gg.drawLine(exes[3],332,xt+20,332) ;
                 off1Gg.drawLine(exes[3],327,exes[3],337) ;
                 off1Gg.setColor(Color.blue) ;
                 off1Gg.fillRect(270,0,30,300) ;
                 off1Gg.setColor(Color.yellow) ;
                 ylabel = (int) (fact*(-h2/2.0)) + yt ;
                 off1Gg.drawString("H2",270,ylabel) ;
                 off1Gg.drawLine(270,yt,280,yt) ;
                 off1Gg.drawLine(275,yt,275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,whys[1]) ;
                 off1Gg.drawLine(270,whys[1],280,whys[1]) ;
                 ylabel = (int) (fact*(-h1/2.0 -h2)) + yt ;
                 off1Gg.drawString("H1",270,ylabel) ;
                 off1Gg.drawLine(275,whys[1],275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,whys[0]) ;
                 off1Gg.drawLine(270,whys[0],280,whys[0]) ;
              }
              if (ktype == 1) {  // delta
                 off1Gg.setColor(Color.white) ;
                 exes[0] = (int) (fact*(0.0)) + xt ;
                 whys[0] = (int) (fact*(-h1 - h2)) + yt ;
                 exes[1] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[1] = (int) (fact*(-h2)) + yt ;
                 exes[2] = (int) (fact*(0.0)) + xt ;
                 whys[2] = (int) (fact*(0.0)) + yt ;
                 exes[3] = (int) (fact*(w1/2.0)) + xt ;
                 whys[3] = (int) (fact*(-h2)) + yt ;
                 exes[4] = (int) (fact*(0.0)) + xt ;
                 whys[4] = (int) (fact*(-h1 - h2)) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;
                 off1Gg.setColor(Color.black) ;
                 off1Gg.drawLine(exes[0],whys[0]+2,exes[1]+2,whys[1]) ;
                 off1Gg.drawLine(exes[0],whys[0]+2,exes[3]-2,whys[3]) ;
               //Labels
                 off1Gg.setColor(Color.yellow) ;
                 off1Gg.drawString("W1",xt-5,337) ;
                 off1Gg.drawLine(exes[1],327,exes[1],337) ;
                 off1Gg.drawLine(exes[1],332,xt-20,332) ;
                 off1Gg.drawLine(exes[3],332,xt+20,332) ;
                 off1Gg.drawLine(exes[3],327,exes[3],337) ;
                 off1Gg.setColor(Color.blue) ;
                 off1Gg.fillRect(270,0,30,300) ;
                 off1Gg.setColor(Color.yellow) ;
                 ylabel = (int) (fact*(-h2/2.0)) + yt ;
                 off1Gg.drawString("H2",270,ylabel) ;
                 off1Gg.drawLine(270,yt,280,yt) ;
                 off1Gg.drawLine(275,yt,275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,whys[1]) ;
                 off1Gg.drawLine(270,whys[1],280,whys[1]) ;
                 ylabel = (int) (fact*(-h1/2.0 -h2)) + yt ;
                 off1Gg.drawString("H1",270,ylabel) ;
                 off1Gg.drawLine(275,whys[1],275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,whys[0]) ;
                 off1Gg.drawLine(270,whys[0],280,whys[0]) ;
              }
              if (ktype == 2) {  // sled
                 off1Gg.setColor(Color.white) ;
                 exes[0] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[0] = (int) (fact*(-h1 - h2)) + yt ;
                 exes[1] = (int) (fact*(-w2/2.0)) + xt ;
                 whys[1] = (int) (fact*(-h2)) + yt ;
                 exes[2] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[2] = (int) (fact*(0.0)) + yt ;
                 exes[3] = (int) (fact*(w1/2.0)) + xt ;
                 whys[3] = (int) (fact*(0.0)) + yt ;
                 exes[4] = (int) (fact*(w2/2.0)) + xt ;
                 whys[4] = (int) (fact*(-h2)) + yt ;
                 exes[5] = (int) (fact*(w1/2.0)) + xt ;
                 whys[5] = (int) (fact*(-h1 - h2)) + yt ;
                 exes[6] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[6] = (int) (fact*(-h1 - h2)) + yt ;
                 off1Gg.fillPolygon(exes,whys,7) ;
                 off1Gg.setColor(Color.black) ;
                 off1Gg.drawLine(exes[0],whys[0],exes[2],whys[2]) ;
                 off1Gg.drawLine(exes[3],whys[3],exes[5],whys[5]) ;
               //Labels
                 off1Gg.setColor(Color.yellow) ;
                 off1Gg.drawString("W1",xt-5,337) ;
                 off1Gg.drawLine(exes[0],327,exes[0],337) ;
                 off1Gg.drawLine(exes[0],332,xt-20,332) ;
                 off1Gg.drawLine(exes[5],332,xt+20,332) ;
                 off1Gg.drawLine(exes[5],327,exes[5],337) ;

                 off1Gg.drawString("W2",xt-5,350) ;
                 off1Gg.drawLine(exes[1],340,exes[1],355) ;
                 off1Gg.drawLine(exes[1],345,xt-20,345) ;
                 off1Gg.drawLine(exes[4],345,xt+20,345) ;
                 off1Gg.drawLine(exes[4],340,exes[4],355) ;

                 off1Gg.setColor(Color.blue) ;
                 off1Gg.fillRect(270,0,30,300) ;
                 off1Gg.setColor(Color.yellow) ;
                 ylabel = (int) (fact*(-h2/2.0)) + yt ;
                 off1Gg.drawString("H2",270,ylabel) ;
                 off1Gg.drawLine(270,yt,280,yt) ;
                 off1Gg.drawLine(275,yt,275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,whys[4]) ;
                 off1Gg.drawLine(270,whys[4],280,whys[4]) ;
                 ylabel = (int) (fact*(-h1/2.0 -h2)) + yt ;
                 off1Gg.drawString("H1",270,ylabel) ;
                 off1Gg.drawLine(275,whys[4],275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,whys[0]) ;
                 off1Gg.drawLine(270,whys[0],280,whys[0]) ;
              }
              if (ktype == 3) {  // box
                 off1Gg.setColor(Color.white) ;
                 exes[0] = (int) (fact*(w1/2.0)) + xt ;
                 whys[0] = (int) (fact*(-h1)) + yt ;
                 exes[1] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[1] = (int) (fact*(-h1)) + yt ;
                 exes[2] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[2] = (int) (fact*(0.0)) + yt ;
                 exes[3] = (int) (fact*(w1/2.0)) + xt ;
                 whys[3] = (int) (fact*(0.0)) + yt ;
                 exes[4] = (int) (fact*(w1/2.0)) + xt ;
                 whys[4] = (int) (fact*(-h1)) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;

                 exes[0] = (int) (fact*(w1/2.0)) + xt ;
                 whys[0] = (int) (fact*(-h1 -h2 -h1)) + yt ;
                 exes[1] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[1] = (int) (fact*(-h1 -h2 -h1)) + yt ;
                 exes[2] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[2] = (int) (fact*(-h1 -h2)) + yt ;
                 exes[3] = (int) (fact*(w1/2.0)) + xt ;
                 whys[3] = (int) (fact*(-h1 -h2)) + yt ;
                 exes[4] = (int) (fact*(w1/2.0)) + xt ;
                 whys[4] = (int) (fact*(-h1 -h2 -h1)) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;
   
               //sticks
                 off1Gg.setColor(Color.black) ;
                 off1Gg.drawLine(exes[1],yt,exes[1],whys[1]) ;
                 off1Gg.drawLine(xt,yt,xt,whys[1]) ;
                 off1Gg.drawLine(exes[4],yt,exes[4],whys[4]) ;
               //Labels
                 off1Gg.setColor(Color.yellow) ;
                 off1Gg.drawString("W1",xt-5,337) ;
                 off1Gg.drawLine(exes[1],327,exes[1],337) ;
                 off1Gg.drawLine(exes[1],332,xt-20,332) ;
                 off1Gg.drawLine(exes[3],332,xt+20,332) ;
                 off1Gg.drawLine(exes[3],327,exes[3],337) ;
              //   off1Gg.drawString("W2",xt-5,350) ;
                 ylabel = (int) (fact*(-h1/2.0)) + yt ;
                 ylabel2 = (int) (fact*(-h1)) + yt ;
                 off1Gg.setColor(Color.blue) ;
                 off1Gg.fillRect(270,0,30,300) ;
                 off1Gg.setColor(Color.yellow) ;
                 off1Gg.drawString("H1",270,ylabel) ;
                 off1Gg.drawLine(270,yt,280,yt) ;
                 off1Gg.drawLine(275,yt,275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,ylabel2) ;
                 off1Gg.drawLine(270,ylabel2,280,ylabel2) ;
                 ylabel = (int) (fact*(-h1 -h2/2.0)) + yt ;
                 off1Gg.drawString("H2",270,ylabel) ;
                 off1Gg.drawLine(275,ylabel2,275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,whys[3]) ;
                 off1Gg.drawLine(270,whys[3],280,whys[3]) ;
                 ylabel = (int) (fact*(-h1/2.0 -h2 -h1)) + yt ;
                 off1Gg.drawString("H1",270,ylabel) ;
                 off1Gg.drawLine(275,whys[3],275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,whys[0]) ;
                 off1Gg.drawLine(270,whys[0],280,whys[0]) ;
              }
              if (ktype == 4) {  // winged box
                 off1Gg.setColor(Color.white) ;
                 exes[0] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[0] = (int) (fact*(-h1 -h2 -h1)) + yt ;
                 exes[1] = (int) (fact*(-w2/2.0)) + xt ;
                 whys[1] = (int) (fact*(-h1 -h2)) + yt ;
                 exes[2] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[2] = (int) (fact*(0.0)) + yt ;
                 exes[3] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[3] = (int) (fact*(-h1 -h2 -h1)) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;

                 exes[0] = (int) (fact*(w1/2.0)) + xt ;
                 whys[0] = (int) (fact*(-h1 -h2 -h1)) + yt ;
                 exes[1] = (int) (fact*(w2/2.0)) + xt ;
                 whys[1] = (int) (fact*(-h1 -h2)) + yt ;
                 exes[2] = (int) (fact*(w1/2.0)) + xt ;
                 whys[2] = (int) (fact*(0.0)) + yt ;
                 exes[3] = (int) (fact*(w1/2.0)) + xt ;
                 whys[3] = (int) (fact*(-h1 -h2 -h1)) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;

                 exes[0] = (int) (fact*(w1/2.0)) + xt ;
                 whys[0] = (int) (fact*(-h1)) + yt ;
                 exes[1] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[1] = (int) (fact*(-h1)) + yt ;
                 exes[2] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[2] = (int) (fact*(0.0)) + yt ;
                 exes[3] = (int) (fact*(w1/2.0)) + xt ;
                 whys[3] = (int) (fact*(0.0)) + yt ;
                 exes[4] = (int) (fact*(w1/2.0)) + xt ;
                 whys[4] = (int) (fact*(-h1)) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;
   
                 exes[0] = (int) (fact*(w1/2.0)) + xt ;
                 whys[0] = (int) (fact*(-h1 -h2 -h1)) + yt ;
                 exes[1] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[1] = (int) (fact*(-h1 -h2 -h1)) + yt ;
                 exes[2] = (int) (fact*(-w1/2.0)) + xt ;
                 whys[2] = (int) (fact*(-h1 -h2)) + yt ;
                 exes[3] = (int) (fact*(w1/2.0)) + xt ;
                 whys[3] = (int) (fact*(-h1 -h2)) + yt ;
                 exes[4] = (int) (fact*(w1/2.0)) + xt ;
                 whys[4] = (int) (fact*(-h1 -h2 -h1)) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;

               //sticks
                 xlabel = (int) (fact*(-w2/2.0)) + xt ;
                 xlabel2 = (int) (fact*(w2/2.0)) + xt ;
                 ylabel2 = (int) (fact*(-h1 -h2)) + yt ;
                 off1Gg.setColor(Color.black) ;
                 off1Gg.drawLine(exes[1],yt,exes[1],whys[1]) ;
                 off1Gg.drawLine(xt,yt,xt,whys[1]) ;
                 off1Gg.drawLine(exes[4],yt,exes[4],whys[4]) ;
                 off1Gg.drawLine(xlabel,ylabel2,xlabel2,ylabel2) ;
            //Labels
                 off1Gg.setColor(Color.yellow) ;
                 off1Gg.drawString("W1",xt-5,337) ;
                 off1Gg.drawLine(exes[1],327,exes[1],337) ;
                 off1Gg.drawLine(exes[1],332,xt-20,332) ;
                 off1Gg.drawLine(exes[3],332,xt+20,332) ;
                 off1Gg.drawLine(exes[3],327,exes[3],337) ;

                 off1Gg.drawString("W2",xt-5,350) ;
                 off1Gg.drawLine(xlabel,340,xlabel,355) ;
                 off1Gg.drawLine(xlabel,345,xt-20,345) ;
                 off1Gg.drawLine(xlabel2,345,xt+20,345) ;
                 off1Gg.drawLine(xlabel2,340,xlabel2,355) ;

                 off1Gg.setColor(Color.blue) ;
                 off1Gg.fillRect(270,0,30,300) ;
                 off1Gg.setColor(Color.yellow) ;
                 ylabel = (int) (fact*(-h1/2.0)) + yt ;
                 ylabel2 = (int) (fact*(-h1)) + yt ;
                 off1Gg.drawString("H1",270,ylabel) ;
                 off1Gg.drawLine(270,yt,280,yt) ;
                 off1Gg.drawLine(275,yt,275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,ylabel2) ;
                 off1Gg.drawLine(270,ylabel2,280,ylabel2) ;
                 ylabel = (int) (fact*(-h1 -h2/2.0)) + yt ;
                 off1Gg.drawString("H2",270,ylabel) ;
                 off1Gg.drawLine(275,ylabel2,275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,whys[3]) ;
                 off1Gg.drawLine(270,whys[3],280,whys[3]) ;
                 ylabel = (int) (fact*(-h1/2.0 -h2 -h1)) + yt ;
                 off1Gg.drawString("H1",270,ylabel) ;
                 off1Gg.drawLine(275,whys[3],275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,whys[0]) ;
                 off1Gg.drawLine(270,whys[0],280,whys[0]) ;
              }
              if (ktype == 5) {  // twin-trapezoid
                 if (fold == -1) {
                    off1Gg.setColor(Color.white) ;
                    exes[0] = (int) (fact*(-h2/2.0)) + xt ;
                    whys[0] = (int) (fact*(-h1)) + yt ;
                    exes[1] = (int) (fact*(h2/2.0)) + xt ;
                    whys[1] = (int) (fact*(-h1)) + yt ;
                    exes[2] = (int) (fact*(h2/2.0)) + xt ;
                    whys[2] = (int) (fact*(0.0)) + yt ;
                    exes[3] = (int) (fact*(-h2/2.0)) + xt ;
                    whys[3] = (int) (fact*(0.0)) + yt ;
                    exes[4] = (int) (fact*(-h2/2.0)) + xt ;
                    whys[4] = (int) (fact*(-h1)) + yt ;
                    off1Gg.fillPolygon(exes,whys,4) ;
                    off1Gg.setColor(Color.lightGray) ;
                    exes[0] = (int) (fact*(-w2/2.0)) + xt ;
                    whys[0] = (int) (fact*(0.0)) + yt ;
                    exes[1] = (int) (fact*(w2/2.0)) + xt ;
                    whys[1] = (int) (fact*(0.0)) + yt ;
                    exes[2] = (int) (fact*(w1/2.0)) + xt ;
                    whys[2] = (int) (fact*(-h1)) + yt ;
                    exes[3] = (int) (fact*(-w1/2.0)) + xt ;
                    whys[3] = (int) (fact*(-h1)) + yt ;
                    exes[4] = (int) (fact*(-w2/2.0)) + xt ;
                    whys[4] = (int) (fact*(0.0)) + yt ;
                    off1Gg.fillPolygon(exes,whys,4) ;
                    off1Gg.setColor(Color.black) ;
                    off1Gg.drawLine(xt,whys[0],xt,whys[2]) ;
                  //Labels
                    off1Gg.setColor(Color.yellow) ;
                    off1Gg.drawString("W2",xt-5,337) ;
                    off1Gg.drawLine(exes[0],327,exes[0],337) ;
                    off1Gg.drawLine(exes[0],332,xt-20,332) ;
                    off1Gg.drawLine(exes[1],332,xt+20,332) ;
                    off1Gg.drawLine(exes[1],327,exes[1],337) ;
   
                    xlabel = (int) (fact*(-h2/2.0)) + xt ;
                    xlabel2 = (int) (fact*(h2/2.0)) + xt ;
                    off1Gg.drawString("H2",xt-5,350) ;
                    off1Gg.drawLine(xlabel,340,xlabel,355) ;
                    off1Gg.drawLine(xlabel,345,xt-20,345) ;
                    off1Gg.drawLine(xlabel2,345,xt+20,345) ;
                    off1Gg.drawLine(xlabel2,340,xlabel2,355) ;

                    xlabel = (int) (fact*(-w1/2.0)) + xt ;
                    xlabel2 = (int) (fact*(w1/2.0)) + xt ;
                    ylabel = (int) (fact*(-h1)) - 10 + yt ;
                    off1Gg.drawString("W1",xt-5,ylabel) ;
                    off1Gg.drawLine(xlabel,ylabel-5,xlabel,ylabel+5) ;
                    off1Gg.drawLine(xlabel,ylabel,xt-20,ylabel) ;
                    off1Gg.drawLine(xlabel2,ylabel,xt+20,ylabel) ;
                    off1Gg.drawLine(xlabel2,ylabel-5,xlabel2,ylabel+5) ;
   
                    off1Gg.setColor(Color.blue) ;
                    off1Gg.fillRect(270,0,30,300) ;
                    off1Gg.setColor(Color.yellow) ;
                    ylabel = (int) (fact*(-h1/2.0)) + yt ;
                    off1Gg.drawString("H1",270,ylabel) ;
                    off1Gg.drawLine(270,whys[2],280,whys[2]) ;
                    off1Gg.drawLine(275,whys[2],275,ylabel-12) ;
                    off1Gg.drawLine(275,ylabel+12,275,whys[0]) ;
                    off1Gg.drawLine(270,whys[0],280,whys[0]) ;
                 }
                 if (fold == 1) {
                    off1Gg.setColor(Color.white) ;
                    exes[0] = (int) (fact*(0.0)) + xt ;
                    whys[0] = (int) (fact*(-lkite)) + yt ;
                    exes[1] = (int) (fact*(lengstk/2.0)) + xt ;
                    whys[1] = (int) (fact*(-lkite + ((h2 - w1)/2.0) * Math.cos(convdr*beta))) + yt ;
                    exes[2] = exes[1] - (int) (fact*(h1 * Math.cos(convdr*beta))) ;
                    whys[2] = whys[1] + (int) (fact*(h1 * Math.sin(convdr*beta))) ;
                    exes[3] = (int) (fact*(0.0)) + xt ;
                    whys[3] = (int) (fact*(0.0)) + yt ;
                    exes[5] = (int) (fact*(-lengstk/2.0)) + xt ;
                    whys[5] = whys[1] ;
                    exes[4] = exes[5] + (int) (fact*(h1 * Math.cos(convdr*beta))) ;
                    whys[4] = whys[2] ;
                    exes[6] = (int) (fact*(0.0)) + xt ;
                    whys[6] = (int) (fact*(-lkite)) + yt ;
                    off1Gg.fillPolygon(exes,whys,6) ;
                    off1Gg.setColor(Color.black) ;
                    off1Gg.drawLine(xt,whys[0],xt,whys[3]) ;
                 }
              }
              if (ktype == 6) {  // tumbleweed
                 off1Gg.setColor(Color.white) ;
                 exes[0] = (int) (fact*(-h1)) + xt ;
                 whys[0] = (int) (fact*(-2.0*h1)) + yt ;
                 exes[1] = (int) (fact*(2.0*h1)) ;
                 off1Gg.fillArc(exes[0],whys[0],exes[1],exes[1],0,360) ;

               //sticks
                 exes[0] = (int) (fact*(0.0)) + xt ;
                 whys[0] = (int) (fact*(0.0)) + yt ;
                 exes[1] = (int) (fact*(0.0)) + xt ;
                 whys[1] = (int) (fact*(-2.0 * h1)) + yt ;
                 off1Gg.setColor(Color.black) ;
                 off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
                 exes[0] = (int) (fact*(-h1)) + xt ;
                 whys[0] = (int) (fact*(-h1)) + yt ;
                 exes[1] = (int) (fact*(h1)) + xt ;
                 whys[1] = (int) (fact*(-h1)) + yt ;
                 off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
               // label
                 off1Gg.setColor(Color.blue) ;
                 off1Gg.fillRect(270,0,30,300) ;
                 off1Gg.setColor(Color.yellow) ;
                 ylabel = (int) (fact*(-h1/2.0)) + yt ;
                 ylabel2 = (int) (fact*(-h1)) + yt ;
                 off1Gg.drawString("H1",270,ylabel) ;
                 off1Gg.drawLine(270,yt,280,yt) ;
                 off1Gg.drawLine(275,yt,275,ylabel+12) ;
                 off1Gg.drawLine(275,ylabel-12,275,ylabel2) ;
                 off1Gg.drawLine(270,ylabel2,280,ylabel2) ;
              }
       // draw tail
              exes[0] = (int) (fact*(0.0)) + xt ;
              whys[0] = (int) (fact*(0.0)) + yt ;
              exes[1] = (int) (fact*(0.0)) + xt ;
              whys[1] = (int) (fact*(0.0 + ltail)) + yt ;
              off1Gg.setColor(Color.green) ;
              off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              if (ktype < 5 ) {
         // cg location
                 off1Gg.setColor(Color.red) ;
                 exes[0] = (int) (fact*(-w1/2.0)) -10 + xt ;
                 whys[0] = (int) (fact*(-cg)) + yt ;
                 off1Gg.fillOval(xt -3,whys[0]- 3,6,6);
                 off1Gg.drawString("Cg",exes[0],whys[0]) ;
         // cp location
                 off1Gg.setColor(Color.green) ;
                 whys[0] = (int) (fact*(-cp)) + yt ;
                 off1Gg.fillOval(xt -3,whys[0]- 3,6,6);
                 off1Gg.drawString("Cp",exes[0],whys[0]) ;
              }
              if (ktype == 5 && fold == 1) {
         // cg location
                 off1Gg.setColor(Color.red) ;
                 exes[0] = (int) (fact*(-w1/2.0)) -10 + xt ;
                 whys[0] = (int) (fact*(-cg)) + yt ;
                 off1Gg.fillOval(xt -3,whys[0]- 3,6,6);
                 off1Gg.drawString("Cg",exes[0],whys[0]) ;
         // cp location
                 off1Gg.setColor(Color.green) ;
                 whys[0] = (int) (fact*(-cp)) + yt ;
                 off1Gg.fillOval(xt -3,whys[0]- 3,6,6);
                 off1Gg.drawString("Cp",exes[0],whys[0]) ;
              }
              if (ktype == 6 ) {  // tumbleweed
         // cg location
                 off1Gg.setColor(Color.red) ;
                 exes[0] = (int) (fact*(0.0)) + xt ;
                 whys[0] = (int) (fact*(-cg)) + yt ;
                 off1Gg.fillOval(xt -3,whys[0]- 3,6,6);
                 off1Gg.drawString("Cg",exes[0] + 5,whys[0]) ;
         // cp location
                 off1Gg.setColor(Color.green) ;
                 whys[0] = (int) (fact*(-cp)) + yt ;
                 off1Gg.fillOval(xt -3,whys[0]- 3,6,6);
                 off1Gg.drawString("Cp",exes[0] + 5,whys[0]) ;
              }
           }
  // SIDE VIEW
           if (viewflg == 1) { 
              off1Gg.setColor(Color.blue) ;
              off1Gg.fillRect(0,0,500,500) ;
              if (ktype == 0) {  // diamond
                 off1Gg.setColor(Color.white) ;
                 exes[0] = (int) (fact*(0.0)) + xt ;
                 whys[0] = (int) (fact*(0.0)) + yt ;
                 exes[1] = (int) (fact*(-lkite * Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(-lkite * Math.sin(angk))) + yt ;
                 exes[2] = (int) (fact*(
                    -h2 * Math.cos(angk) + 2.0*Math.sin(angk))) + xt ;
                 whys[2] = (int) (fact*(
                    -h2 * Math.sin(angk) - 2.0 * Math.cos(angk))) + yt ;
                 exes[3] = (int) (fact*(0.0)) + xt ;
                 whys[3] = (int) (fact*(0.0)) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;
              }
              if (ktype == 1) {  // delta
                 off1Gg.setColor(Color.white) ;
                 exes[0] = (int) (fact*(0.0)) + xt ;
                 whys[0] = (int) (fact*(0.0)) + yt ;
                 exes[1] = (int) (fact*(-lkite * Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(-lkite * Math.sin(angk))) + yt ;
                 exes[2] = (int) (fact*(
                    -h2 * Math.cos(angk) + 2.0*Math.sin(angk))) + xt ;
                 whys[2] = (int) (fact*(
                    -h2 * Math.sin(angk) - 2.0 * Math.cos(angk))) + yt ;
                 exes[3] = (int) (fact*(0.0)) + xt ;
                 whys[3] = (int) (fact*(0.0)) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;
                 off1Gg.setColor(Color.black) ;
                 off1Gg.drawLine(exes[1],whys[1]+2,exes[2]+2,whys[2]) ;
              }
              if (ktype == 2) {  // sled
                 off1Gg.setColor(Color.white) ;
                 exes[0] = (int) (fact*(0.0)) + xt ;
                 whys[0] = (int) (fact*(0.0)) + yt ;
                 exes[1] = (int) (fact*(-lkite * Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(-lkite * Math.sin(angk))) + yt ;
                 exes[2] = (int) (fact*(
                    -h2 * Math.cos(angk) - xbr*Math.sin(angk))) + xt ;
                 whys[2] = (int) (fact*(
                    -h2 * Math.sin(angk) + xbr * Math.cos(angk))) + yt ;
                 exes[3] = (int) (fact*(0.0)) + xt ;
                 whys[3] = (int) (fact*(0.0)) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;
              }
              if (ktype == 3) {  // box 
                 off1Gg.setColor(Color.white) ;
    // lower panel
                 exes[0] = (int) (fact*(-kbase * Math.sin(angk))) + xt ;
                 whys[0] = (int) (fact*(kbase * Math.cos(angk))) + yt ;
                 exes[1] = (int) (fact*(-kbase * Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(kbase * Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 exes[3] = (int) (fact*(+kbase*Math.sin(angk))) + xt ;
                 whys[3] = (int) (fact*(-kbase*Math.cos(angk))) + yt ;
                 exes[2] = (int) (fact*(+kbase*Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[2] = (int) (fact*(-kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;
    // upper panel
                 exes[0] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                       - kbase*Math.sin(angk))) + xt ;
                 whys[0] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                       + kbase*Math.cos(angk))) + yt ;
                 exes[1] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        - kbase*Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        + kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 exes[3] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        + kbase*Math.sin(angk))) + xt ;
                 whys[3] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        - kbase*Math.cos(angk))) + yt ;
                 exes[2] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        + kbase*Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[2] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        - kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;
          //sticks
                 exes[0] = (int) (fact*(-kbase*Math.sin(angk))) + xt ;
                 whys[0] = (int) (fact*(kbase*Math.cos(angk))) + yt ;
                 exes[1] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        - kbase*Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        + kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 off1Gg.setColor(Color.black) ;
                 off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
                 exes[0] = (int) (fact*(0.0)) + xt ;
                 whys[0] = (int) (fact*(0.0)) + yt ;
                 exes[1] = (int) (fact*(-lkite*Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(-lkite*Math.sin(angk))) + yt ;
                 off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
                 exes[3] = (int) (fact*(+kbase*Math.sin(angk))) + xt ;
                 whys[3] = (int) (fact*(-kbase*Math.cos(angk))) + yt ;
                 exes[2] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        + kbase*Math.sin(angk)
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[2] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        - kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 off1Gg.drawLine(exes[2],whys[2],exes[3],whys[3]) ;
              }
              if (ktype == 4) {  // winged box
                 off1Gg.setColor(Color.white) ;
    // lower panel
                 exes[0] = (int) (fact*(-kbase * Math.sin(angk))) + xt ;
                 whys[0] = (int) (fact*(kbase * Math.cos(angk))) + yt ;
                 exes[1] = (int) (fact*(-kbase * Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(kbase * Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 exes[3] = (int) (fact*(+kbase*Math.sin(angk))) + xt ;
                 whys[3] = (int) (fact*(-kbase*Math.cos(angk))) + yt ;
                 exes[2] = (int) (fact*(+kbase*Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[2] = (int) (fact*(-kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;
    // upper panel
                 exes[0] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                       - kbase*Math.sin(angk))) + xt ;
                 whys[0] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                       + kbase*Math.cos(angk))) + yt ;
                 exes[1] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        - kbase*Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        + kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 exes[3] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        + kbase*Math.sin(angk))) + xt ;
                 whys[3] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        - kbase*Math.cos(angk))) + yt ;
                 exes[2] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        + kbase*Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[2] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        - kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 off1Gg.fillPolygon(exes,whys,4) ;
          //sticks
                 exes[0] = (int) (fact*(-kbase*Math.sin(angk))) + xt ;
                 whys[0] = (int) (fact*(kbase*Math.cos(angk))) + yt ;
                 exes[1] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        - kbase*Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        + kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 off1Gg.setColor(Color.black) ;
                 off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
                 exes[0] = (int) (fact*(0.0)) + xt ;
                 whys[0] = (int) (fact*(0.0)) + yt ;
                 exes[1] = (int) (fact*(-lkite*Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(-lkite*Math.sin(angk))) + yt ;
                 off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
                 exes[3] = (int) (fact*(+kbase*Math.sin(angk))) + xt ;
                 whys[3] = (int) (fact*(-kbase*Math.cos(angk))) + yt ;
                 exes[2] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        + kbase*Math.sin(angk)
                                        - h1 * Math.cos(angk))) + xt ;
                 whys[2] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        - kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                 off1Gg.drawLine(exes[2],whys[2],exes[3],whys[3]) ;
              }
              if (ktype == 5) {  // twin-trapezoid
                 off1Gg.setColor(Color.white) ;
                 exes[0] = (int) (fact*(0.0)) + xt ;
                 whys[0] = (int) (fact*(0.0)) + yt ;
                 exes[1] = (int) (fact*(-lkite * Math.cos(angk))) + xt ;
                 whys[1] = (int) (fact*(-lkite * Math.sin(angk))) + yt ;
                 exes[2] = exes[1] - (int) (fact*((w1/2.0) * Math.sin(angk))) ;
                 whys[2] = whys[1] + (int) (fact*((w1/2.0) * Math.cos(angk))) ;
                 exes[3] = exes[0] - (int) (fact*((w2/2.0) * Math.sin(angk))) ;
                 whys[3] = whys[0] + (int) (fact*((w2/2.0) * Math.cos(angk))) ;
                 exes[4] = (int) (fact*(0.0)) + xt ;
                 whys[4] = (int) (fact*(0.0)) + yt ;
                 off1Gg.setColor(Color.lightGray) ;
                 off1Gg.fillPolygon(exes,whys,4) ;
              }
              if (ktype == 6) {  // tumbleweed
                 cang = Math.cos(angk) ;
                 sang = Math.sin(angk) ;
                 off1Gg.setColor(Color.white) ;
                 exes[0] = (int) (fact*(-h1 * cang - h1)) + xt ;
                 whys[0] = (int) (fact*(-h1 * sang - h1)) + yt ;
                 exes[1] = (int) (fact*(2.0*h1)) ;
                 off1Gg.fillOval(exes[0],whys[0],exes[1],exes[1]) ;

               //sticks
                 exes[0] = (int) (fact*(0.0)) + xt ;
                 whys[0] = (int) (fact*(0.0)) + yt ;
                 exes[1] = (int) (fact*(-lkite * cang)) + xt ;
                 whys[1] = (int) (fact*(-lkite * sang)) + yt ;
                 off1Gg.setColor(Color.black) ;
                 off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
                 exes[0] = (int) (fact*(-(h1 * cang - h1 * sang))) + xt ;
                 whys[0] = (int) (fact*(-(h1 * sang + h1 * cang))) + yt ;
                 exes[1] = (int) (fact*(-(h1 * cang + h1 * sang))) + xt ;
                 whys[1] = (int) (fact*(-(h1 * sang - h1 * cang))) + yt ;
                 off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              }
      // draw tail
              exes[0] = (int) (fact*(0.0)) + xt ;
              whys[0] = (int) (fact*(0.0)) + yt ;
              off1Gg.setColor(Color.cyan) ;
              for (i=1; i<=8; ++ i) {
                ripple = (1 - ((i/2) * 2 / i)) * 1. ;
                if (i == 3) ripple = - ripple ;
                if (i == 7) ripple = - ripple ;
                exes[i] = (int) (fact*(.125*i*ltail * Math.cos(angk))) + xt ;
                whys[i] = (int) (fact*(.125*i*ltail * Math.sin(angk) + ripple)) + yt ;
                off1Gg.drawLine(exes[i],whys[i],exes[i-1],whys[i-1]) ;
              }
              off1Gg.drawString("T",exes[3]-5,whys[3]+15) ;
     // draw bridle
              exes[4] = (int) (fact*(-300. * Math.sin(angl))) + xt ;
              whys[4] = (int) (fact*(300. * Math.cos(angl))) + yt ;
              off1Gg.setColor(Color.white) ;
              switch (ktype) { 
                 case 0: {    // diamond 
                    exes[0] = (int) (fact*(0.0)) + xt ;
                    whys[0] = (int) (fact*(0.0)) + yt ;
                    exes[1] = (int) (fact*(-lknot * Math.cos(angk - anga))) + xt ;
                    whys[1] = (int) (fact*(-lknot * Math.sin(angk - anga))) + yt ;
                    exes[2] = (int) (fact*(-lkite * Math.cos(angk))) + xt ;
                    whys[2] = (int) (fact*(-lkite * Math.sin(angk))) + yt ;
                    off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
                    off1Gg.drawLine(exes[1],whys[1],exes[2],whys[2]) ;
                    off1Gg.drawString("B",exes[1]-20,whys[1]) ;
                    off1Gg.fillOval(exes[1]-3,whys[1]-3,6,6) ;
                    off1Gg.setColor(Color.magenta) ;
                    off1Gg.drawLine(exes[1],whys[1],exes[4],whys[4]) ;
                    off1Gg.setColor(Color.yellow) ;
                    off1Gg.drawLine(exes[0],whys[0]+10,exes[1],whys[1]+10) ;
                    off1Gg.drawString("K",(exes[0]+exes[1])/2,((whys[0]+whys[1])/2)+30) ;
                    off1Gg.drawLine(exes[0],whys[0]+5,exes[0],whys[0]+15) ;
                    off1Gg.drawLine(exes[1],whys[1]+5,exes[1],whys[1]+15) ;
                    break;
                 }
                 case 1: {    // delta
                    exes[0] = (int) (fact*(0.0)) + xt ;
                    whys[0] = (int) (fact*(0.0)) + yt ;
                    exes[1] = (int) (fact*(-lknot * Math.cos(angk - anga))) + xt ;
                    whys[1] = (int) (fact*(-lknot * Math.sin(angk - anga))) + yt ;
                    exes[2] = (int) (fact*(-lkite * Math.cos(angk))) + xt ;
                    whys[2] = (int) (fact*(-lkite * Math.sin(angk))) + yt ;
                    exes[3] = (int) (fact*(0.0)) + xt ;
                    whys[3] = (int) (fact*(0.0)) + yt ;
                    off1Gg.setColor(Color.magenta) ;
                    off1Gg.fillPolygon(exes,whys,4) ;
                    off1Gg.setColor(Color.white) ;
                    off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
                    off1Gg.drawLine(exes[1],whys[1],exes[2],whys[2]) ;
                    off1Gg.drawString("B",exes[1]-20,whys[1]) ;
                    off1Gg.fillOval(exes[1]-3,whys[1]-3,6,6) ;
                    off1Gg.setColor(Color.magenta) ;
                    off1Gg.drawLine(exes[1],whys[1],exes[4],whys[4]) ;
                    off1Gg.setColor(Color.yellow) ;
                    off1Gg.drawLine(exes[0],whys[0]+10,exes[1],whys[1]+10) ;
                    off1Gg.drawString("K",(exes[0]+exes[1])/2,((whys[0]+whys[1])/2)+30) ;
                    off1Gg.drawLine(exes[0],whys[0]+5,exes[0],whys[0]+15) ;
                    off1Gg.drawLine(exes[1],whys[1]+5,exes[1],whys[1]+15) ;
                    break;
                 }
                 case 2: {    // sled
                    exes[0] = (int) (fact*(
                       -h2 * Math.cos(angk) - xbr*Math.sin(angk))) + xt ;
                    whys[0] = (int) (fact*(
                       -h2 * Math.sin(angk) + xbr * Math.cos(angk))) + yt ;

                    exes[1] = exes[0] -
                             (int) (fact*(lbrid/2.0 * Math.sin(angl)));
                    whys[1] = whys[0] +
                             (int) (fact*(lbrid/2.0 * Math.cos(angl)));
                    off1Gg.setColor(Color.white) ;
                    off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
                    off1Gg.drawString("B/2",exes[1]-10,whys[1]-10) ;
                    off1Gg.fillOval(exes[1]-3,whys[1]-3,6,6) ;
                    off1Gg.setColor(Color.magenta) ;
                    off1Gg.drawLine(exes[1],whys[1],exes[4],whys[4]) ;
                    break;
                 }
                 case 3: {    // box 
                    exes[0] = (int) (fact*(-kbase*Math.sin(angk))) + xt ;
                    whys[0] = (int) (fact*(kbase*Math.cos(angk))) + yt ;
                    exes[1] = (int) (fact*(-kbase*Math.sin(angk)
                                     - lknot * Math.cos(angk - anga))) + xt ;
                    whys[1] = (int) (fact*(kbase*Math.cos(angk)
                                     - lknot * Math.sin(angk - anga))) + yt ;
                    exes[2] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        - kbase*Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                    whys[2] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        + kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                    off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
                    off1Gg.drawLine(exes[1],whys[1],exes[2],whys[2]) ;
                    off1Gg.drawString("B",exes[1]-20,whys[1]) ;
                    off1Gg.fillOval(exes[1]-3,whys[1]-3,6,6) ;
                    off1Gg.setColor(Color.magenta) ;
                    off1Gg.drawLine(exes[1],whys[1],exes[4],whys[4]) ;
                    off1Gg.setColor(Color.yellow) ;
                    off1Gg.drawLine(exes[0],whys[0]+10,exes[1],whys[1]+10) ;
                    off1Gg.drawString("K",(exes[0]+exes[1])/2,((whys[0]+whys[1])/2)+30) ;
                    off1Gg.drawLine(exes[0],whys[0]+5,exes[0],whys[0]+15) ;
                    off1Gg.drawLine(exes[1],whys[1]+5,exes[1],whys[1]+15) ;
                    break;
                 }
                 case 4: {    // winged box 
                    exes[0] = (int) (fact*(-kbase*Math.sin(angk))) + xt ;
                    whys[0] = (int) (fact*(kbase*Math.cos(angk))) + yt ;
                    exes[1] = (int) (fact*(-kbase*Math.sin(angk)
                                     - lknot * Math.cos(angk - anga))) + xt ;
                    whys[1] = (int) (fact*(kbase*Math.cos(angk)
                                     - lknot * Math.sin(angk - anga))) + yt ;
                    exes[2] = (int) (fact*(-(h1+h2)*Math.cos(angk)
                                        - kbase*Math.sin(angk) 
                                        - h1 * Math.cos(angk))) + xt ;
                    whys[2] = (int) (fact*(-(h1+h2)*Math.sin(angk)
                                        + kbase*Math.cos(angk)
                                        - h1 * Math.sin(angk))) + yt ;
                    off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
                    off1Gg.drawLine(exes[1],whys[1],exes[2],whys[2]) ;
                    off1Gg.drawString("B",exes[1]-20,whys[1]) ;
                    off1Gg.fillOval(exes[1]-3,whys[1]-3,6,6) ;
                    off1Gg.setColor(Color.magenta) ;
                    off1Gg.drawLine(exes[1],whys[1],exes[4],whys[4]) ;
                    off1Gg.setColor(Color.yellow) ;
                    off1Gg.drawLine(exes[0],whys[0]+10,exes[1],whys[1]+10) ;
                    off1Gg.drawString("K",(exes[0]+exes[1])/2,((whys[0]+whys[1])/2)+30) ;
                    off1Gg.drawLine(exes[0],whys[0]+5,exes[0],whys[0]+15) ;
                    off1Gg.drawLine(exes[1],whys[1]+5,exes[1],whys[1]+15) ;
                    break;
                 }
                 case 5: {    // twin-trapezoid 
                    exes[0] = (int) (fact*(0.0)) + xt ;
                    whys[0] = (int) (fact*(0.0)) + yt ;
                    exes[1] = (int) (fact*(-lknot * Math.cos(angk - anga))) + xt ;
                    whys[1] = (int) (fact*(-lknot * Math.sin(angk - anga))) + yt ;
                    off1Gg.setColor(Color.magenta) ;
                    off1Gg.fillOval(exes[1]-3,whys[1]-3,6,6) ;
                    off1Gg.drawLine(exes[1],whys[1],exes[4],whys[4]) ;
                    off1Gg.setColor(Color.yellow) ;
                    off1Gg.drawLine(exes[0],whys[0]+10,exes[1],whys[1]+10) ;
                    off1Gg.drawString("K",(exes[0]+exes[1])/2,((whys[0]+whys[1])/2)+30) ;
                    off1Gg.drawLine(exes[0],whys[0]+5,exes[0],whys[0]+15) ;
                    off1Gg.drawLine(exes[1],whys[1]+5,exes[1],whys[1]+15) ;
                    break;
                 }
                 case 6: {    // tumbleweed
                    exes[0] = (int) (fact*(-h1*Math.cos(angk)-h1*Math.cos(angk - anga))) + xt ;
                    whys[0] = (int) (fact*(-h1*Math.sin(angk)-h1*Math.sin(angk - anga))) + yt ;
                    off1Gg.setColor(Color.magenta) ;
                    off1Gg.fillOval(exes[0]-3,whys[0]-3,6,6) ;
                    off1Gg.drawLine(exes[0],whys[0],exes[4],whys[4]) ;
                    break;
                 }
              }
   // compute force vectors
   // weight - cg location
              exes[0] = (int) (fact*(-cg * Math.cos(angk))) + xt ;
              whys[0] = (int) (fact*(-cg * Math.sin(angk))) + yt ;
              exes[1] = (int) (fact*(-cg * Math.cos(angk))) + xt ;
              whys[1] = (int) (fact*(-(cg * Math.sin(angk) - wtplnt*fscale))) + yt ;
              off1Gg.setColor(Color.red) ;
              off1Gg.fillOval(exes[0]-3,whys[0]-3,6,6);
              off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              exes[0] = exes[1] - 3 ;
              whys[0] = whys[1] - 5 ;
              exes[2] = exes[1] + 3 ;
              whys[2] = whys[1] - 5 ;
              exes[3] = exes[1] - 3 ;
              whys[3] = whys[1] - 5 ;
              off1Gg.fillPolygon(exes,whys,4) ;
              off1Gg.drawString("Weight",exes[1]+5,whys[1]-5) ;
   // lift - cp location
              exes[0] = (int) (fact*(-cp * Math.cos(angk))) + xt ;
              whys[0] = (int) (fact*(-cp * Math.sin(angk))) + yt ;
              exes[1] = (int) (fact*(-cp * Math.cos(angk))) + xt ;
              whys[1] = (int) (fact*(-(cp * Math.sin(angk) + lift*fscale))) + yt ;
              off1Gg.setColor(Color.green) ;
              off1Gg.fillOval(exes[0]-3,whys[0]-3,6,6);
              off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              exes[0] = exes[1] - 3 ;
              whys[0] = whys[1] + 5 ;
              exes[2] = exes[1] + 3 ;
              whys[2] = whys[1] + 5 ;
              exes[3] = exes[1] - 3 ;
              whys[3] = whys[1] + 5 ;
              off1Gg.fillPolygon(exes,whys,4) ;
              off1Gg.drawString("Lift",exes[1]+20,whys[1]+10) ;
   // drag - cp location
              exes[0] = (int) (fact*(-cp * Math.cos(angk))) + xt ;
              whys[0] = (int) (fact*(-cp * Math.sin(angk))) + yt ;
              exes[1] = (int) (fact*(-cp * Math.cos(angk) + drag*fscale)) + xt ;
              whys[1] = (int) (fact*(-cp * Math.sin(angk))) + yt ;
              off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              exes[0] = exes[1] - 5 ;
              whys[0] = whys[1] - 3 ;
              exes[2] = exes[1] - 5 ;
              whys[2] = whys[1] + 3 ;
              exes[3] = exes[1] - 5 ;
              whys[3] = whys[1] - 3 ;
              off1Gg.fillPolygon(exes,whys,4) ;
              off1Gg.drawString("Drag",exes[1]-30,whys[1]-5) ;
   // wind vector
              exes[0] = 100 ;
              whys[0] = 275;
              exes[1] = 100 + (int) (5.*wind) ;
              whys[1] = 275 ;
              off1Gg.setColor(Color.green) ;
              off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              exes[0] = exes[1] - 5 ;
              whys[0] = whys[1] - 3 ;
              exes[2] = exes[1] - 5 ;
              whys[2] = whys[1] + 3 ;
              exes[3] = exes[1] - 5 ;
              whys[3] = whys[1] - 3 ;
              off1Gg.fillPolygon(exes,whys,4) ;
              off1Gg.drawString("Wind",exes[1]-30,whys[1]+15) ;
           }

 // FIELD VIEW
           if (viewflg == 2) { 
              if (planet == 0) off1Gg.setColor(Color.cyan) ;
              if (planet == 1) off1Gg.setColor(Color.pink) ;
              off1Gg.fillRect(0,0,500,500) ;

              if (planet == 0) off1Gg.setColor(Color.green) ;
              if (planet == 1) off1Gg.setColor(Color.orange) ;
              off1Gg.fillRect(0,-40 + yt,350,200) ;
              off1Gg.setColor(Color.white) ;
              exes[1] = (int) (fact*fscale*(xline[0])) + xt ;
              whys[1] = (int) (fact*fscale*(-yline[0])) + yt ;
              for (i=1; i<=20; ++i) {
                 exes[0] = exes[1] ;
                 whys[0] = whys[1] ;
                 exes[1] = (int) (fact*fscale*(xline[i])) + xt ;
                 whys[1] = (int) (fact*fscale*(-yline[i])) + yt ;
                 off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              }

              exes[0] = (int) (fact*fscale*(xline[0])) + xt ;
              whys[0] = (int) (fact*fscale*(-yline[0])) + yt ;
              exes[1] = (int) (fact*fscale*(xline[0] -2.0)) + xt ;
              whys[1] = (int) (fact*fscale*(-yline[0] +1.2)) + yt ;
              off1Gg.setColor(Color.black) ;
              off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              exes[1] = (int) (fact*fscale*(xline[0] -3.5)) + xt ;
              whys[1] = (int) (fact*fscale*(-yline[0] + 1.2)) + yt ;
              off1Gg.fillRect(exes[1],whys[1],
                  ((int) (fact*fscale*1.5)),((int) (fact*fscale*3.0)));
              exes[0] = (int) (fact*fscale*(xline[0] -3.5)) + xt ;
              whys[0] = (int) (fact*fscale*(-yline[0] + 4.0)) + yt ;
              exes[1] = (int) (fact*fscale*(xline[0] -3.5)) + xt ;
              whys[1] = (int) (fact*fscale*(-yline[0] + 6.0)) + yt ;
              off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              exes[0] = (int) (fact*fscale*(xline[0] -2.2)) + xt ;
              whys[0] = (int) (fact*fscale*(-yline[0] + 4.0)) + yt ;
              exes[1] = (int) (fact*fscale*(xline[0] -2.2)) + xt ;
              whys[1] = (int) (fact*fscale*(-yline[0] + 6.0)) + yt ;
              off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              exes[1] = (int) (fact*fscale*(xline[0]-3.0)) + xt ;
              whys[1] = (int) (fact*fscale*(-yline[0])) + yt ;
              off1Gg.fillRect(exes[1],whys[1],
                  ((int) (fact*fscale*1.2)),((int) (fact*fscale*1.0)));

              exes[0] = (int) (fact*fscale*(xline[20])) + xt;
              whys[0] = (int) (fact*fscale*(-yline[20])) + yt - 3 ;
              exes[1] = exes[0] + 3 ;
              whys[1] = whys[0] + 3 ;
              exes[2] = exes[0] ;
              whys[2] = whys[0] + 10 ;
              exes[3] = exes[0] - 3 ;
              whys[3] = whys[0] + 3 ;
              exes[4] = exes[0] ;
              whys[4] = whys[0] ;
              off1Gg.setColor(Color.white) ;
              off1Gg.fillPolygon(exes,whys,5) ;

              exes[0] = (int) (fact*fscale*(xline[0])) + xt ;
              whys[0] = (int) (fact*fscale*(-yline[0])) + yt ;
              exes[1] = (int) (fact*fscale*(xline[20])) + xt ;
              whys[1] = whys[0];
              off1Gg.setColor(Color.red) ;
              off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              off1Gg.drawString("X",exes[1]-50,whys[1]+12) ;

              exes[0] = (int) (fact*fscale*(xline[20])) + xt ;
              whys[0] = (int) (fact*fscale*(-yline[0])) + yt ;
              exes[1] = exes[0] ;
              whys[1] = (int) (fact*fscale*(-yline[20])) + yt ;
              off1Gg.setColor(Color.black) ;
              off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              off1Gg.drawString("Y",exes[1]+10,whys[1]+50) ;
   // wind vector
              exes[0] = 100 ;
              whys[0] = 75;
              exes[1] = 100 + (int) (5.*wind) ;
              whys[1] = 75 ;
              off1Gg.setColor(Color.black) ;
              off1Gg.drawLine(exes[0],whys[0],exes[1],whys[1]) ;
              exes[0] = exes[1] - 5 ;
              whys[0] = whys[1] - 3 ;
              exes[2] = exes[1] - 5 ;
              whys[2] = whys[1] + 3 ;
              exes[3] = exes[1] - 5 ;
              whys[3] = whys[1] - 3 ;
              off1Gg.fillPolygon(exes,whys,4) ;
              off1Gg.drawString("Wind",exes[1]-30,whys[1]+15) ;
 // warning 
              if (ground == 1) {
                 off1Gg.setColor(Color.black) ;
                 off1Gg.drawString("Line Touches Ground",50,200) ;
              }
           }

 // unstable design
           if (unstab == 1) {
              off1Gg.setColor(Color.yellow) ;
              off1Gg.fillRect(50,250,200,70) ;
              off1Gg.setColor(Color.red) ;
              off1Gg.fillRect(70,265,160,40) ;
              off1Gg.setColor(Color.white) ;
              off1Gg.drawString("UNSTABLE DESIGN",90,280) ;
              off1Gg.drawString(" OR  TRIM  ",130,295) ;
           }

 // insufficient lift
           if (nolift == 1) {
              off1Gg.setColor(Color.yellow) ;
              off1Gg.fillRect(50,250,200,70) ;
              off1Gg.setColor(Color.red) ;
              off1Gg.fillRect(70,265,160,40) ;
              off1Gg.setColor(Color.white) ;
              off1Gg.drawString("INSUFFICIENT",90,280) ;
              off1Gg.drawString(" LIFT ",130,295) ;
           }

 // zoom widget
           off1Gg.setColor(Color.lightGray) ;
           off1Gg.fillRect(0,10,35,390) ;
           off1Gg.setColor(Color.black) ;
           off1Gg.drawString("Zoom",2,295) ;
           off1Gg.drawLine(15,50,15,275) ;
           off1Gg.fillRect(5,sldloc,20,5) ;

 // scale  widget 
           off1Gg.setColor(Color.lightGray) ;
           off1Gg.fillRect(34,365,400,40) ;
           if (viewflg >= 1) {
              off1Gg.setColor(Color.black) ;
              off1Gg.drawString("Scale",55,385) ;
              off1Gg.drawLine(95,380,225,380) ;
              off1Gg.fillRect(fldloc,370,5,20) ;
           }

 // find button
           off1Gg.setColor(Color.white) ;
           off1Gg.fillRect(2,30,30,15) ;
           off1Gg.setColor(Color.blue) ;
           off1Gg.drawString("Find",3,42) ;

// borders
           off1Gg.setColor(Color.lightGray) ;
           off1Gg.fillRect(0,0,350,25) ;
           off1Gg.setColor(Color.lightGray) ;
           off1Gg.fillRect(325,0,25,450) ;
 // control buttons

           off1Gg.setColor(Color.white) ;
           off1Gg.fillRect(5,5,55,17) ;
           off1Gg.setColor(Color.red) ;
           if (units == 1) {
              off1Gg.drawString("Imperial",10,17) ;
           }
           else {
              off1Gg.drawString("Metric",10,17) ;
           }

           off1Gg.setColor(Color.white) ;
           if (pick == 1) {
              off1Gg.setColor(Color.yellow) ;
           }
           off1Gg.fillRect(65,5,80,17) ;
           off1Gg.setColor(Color.blue) ;
           off1Gg.drawString("Select View->",70,17) ;

           off1Gg.setColor(Color.white) ;
           if (viewflg == 0) {
              off1Gg.setColor(Color.yellow) ;
           }
           off1Gg.fillRect(150,5,50,17) ;
           off1Gg.setColor(Color.blue) ;
           off1Gg.drawString("Front",155,17) ;

           off1Gg.setColor(Color.white) ;
           if (viewflg == 1) {
              off1Gg.setColor(Color.yellow) ;
           }
           off1Gg.fillRect(205,5,40,17) ;
           off1Gg.setColor(Color.blue) ;
           off1Gg.drawString("Side",210,17) ;

           off1Gg.setColor(Color.white) ;
           if (viewflg == 2) {
              off1Gg.setColor(Color.yellow) ;
           }
           off1Gg.fillRect(250,5,50,17) ;
           off1Gg.setColor(Color.blue) ;
           off1Gg.drawString("Field",255,17) ;

      // print button
           off1Gg.setColor(Color.white) ;
           off1Gg.fillRect(2,330,30,20) ;
           off1Gg.setColor(Color.blue) ;
           off1Gg.drawString("Print",3,347) ;

      // reset button
           off1Gg.setColor(Color.white) ;
           off1Gg.fillRect(2,367,40,20) ;
           if (ktype <= 4) {
              off1Gg.setColor(Color.blue) ;
              off1Gg.drawString("Reset",3,382) ;
           }

           if (ktype == 5) {
              off1Gg.setColor(Color.red) ;
              off1Gg.drawString("Fold",3,382) ;
           }

           off1Gg.setColor(Color.black) ;
           off1Gg.drawString("Version 1.5a",250,385) ;

           g.drawImage(offImg1,0,0,this) ;   

       }  // Paint
    }  // Viewer
} // Kite
