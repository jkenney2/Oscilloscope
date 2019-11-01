using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Media;

namespace Oscilloscope
{
    class OscilloscopeHost : Canvas
    {
        DrawingVisual oscilloscopeVisual = null;

        public OscilloscopeHost()
        {
            this.ClipToBounds = true;
            oscilloscopeVisual = new DrawingVisual();
            AddVisualChild(oscilloscopeVisual);
            AddLogicalChild(oscilloscopeVisual);
        }

        protected override int VisualChildrenCount { get { return 1; } }

        protected override Visual GetVisualChild(int index)
        {
            if (index > 0) throw new ArgumentOutOfRangeException("GetVisualChild index out of range");
            return oscilloscopeVisual;
        }

        public void DrawDrawing(Drawing dr)
        {
            using (DrawingContext dc = oscilloscopeVisual.RenderOpen()) dc.DrawDrawing(dr);
        }

    }
}
