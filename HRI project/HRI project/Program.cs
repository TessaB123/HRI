using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Windows;
//using System.Windows.Media;
//using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
//using System.Windows.Data;
//using System.Windows.Documents;
using System.Windows.Input;
//using System.Windows.Navigation;
//using System.Windows.Shapes;
using Microsoft.VisualBasic;
using Microsoft.VisualBasic.FileIO;


namespace HRI_project
{
    class Program
    {
        static void Main(string[] args)
        {
            HRIModel m = new HRIModel();
            HRIController c = new HRIController (m);
        }
    }
}
