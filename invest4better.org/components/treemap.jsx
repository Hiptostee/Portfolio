"use client";
import React, { useEffect, useRef, useState } from "react";
import * as d3 from "d3";
import { useRouter } from "next/navigation";

const Treemap = ({ data, onClickSector }) => {
  const router = useRouter();
  const svgRef = useRef(null);
  const containerRef = useRef(null);
  const tooltipRef = useRef(null);
  const [dimensions, setDimensions] = useState({ width: 800, height: 800 });

  // Resize listener
  useEffect(() => {
    const resizeObserver = new ResizeObserver((entries) => {
      for (let entry of entries) {
        const { width, height } = entry.contentRect;
        setDimensions({
          width: Math.max(width, 300),
          height: Math.max(height, 300),
        });
      }
    });

    if (containerRef.current) {
      resizeObserver.observe(containerRef.current);
    }

    return () => {
      if (containerRef.current) {
        resizeObserver.unobserve(containerRef.current);
      }
    };
  }, []);

  useEffect(() => {
    const { width, height } = dimensions;

    const root = d3
      .hierarchy({ children: data })
      .sum((d) => d.marketCap)
      .sort((a, b) => b.value - a.value);

    d3.treemap().size([width, height]).padding(1)(root);

    const svg = d3
      .select(svgRef.current)
      .attr("viewBox", `0 0 ${width} ${height}`)
      .attr("preserveAspectRatio", "xMidYMid meet");

    svg.selectAll("*").remove(); // Clear previous

    const tooltip = d3
      .select(tooltipRef.current)
      .style("position", "absolute")
      .style("visibility", "hidden")
      .style("background", "white")
      .style("border", "1px solid #333")
      .style("padding", "5px")
      .style("border-radius", "5px")
      .style("color", "#000")
      .style("font-size", "14px");

    const esgScores = data.map((d) => d.esgScore);
    const colorScale = d3
      .scaleLinear()
      .domain([d3.min(esgScores), d3.max(esgScores)])
      .range(["#00b241", "#950202"]);

    const cell = svg
      .selectAll("g")
      .data(root.leaves())
      .enter()
      .append("g")
      .attr("transform", (d) => `translate(${d.x0},${d.y0})`);

    cell
      .append("rect")
      .attr("width", (d) => d.x1 - d.x0)
      .attr("height", (d) => d.y1 - d.y0)
      .style("fill", (d) => colorScale(d.data.esgScore))
      .style("stroke", "#fff")

      .on("mousemove", (event) => {
        tooltip
          .style("top", event.pageY + 10 + "px")
          .style("left", event.pageX + 10 + "px");
      })
      .on("mouseout", function (event, d) {
        tooltip.style("visibility", "hidden");
        d3.select(this)
          .transition()
          .duration(200)
          .style("fill", colorScale(d.data.esgScore));
      })
      .on("click", (event, d) => {
        if (onClickSector) {
          router.push(`/sector?sector=${d.data.sector}`);
        }
      });

    cell
      .append("text")
      .attr("x", 4)
      .attr("y", 14)
      .style("fill", "#fff")
      .style("font-size", "12px")
      .style("pointer-events", "none")
      .each(function (d) {
        const text = d3.select(this);
        const words = d.data.sector.split(/\s+/);
        const boxWidth = d.x1 - d.x0 - 8; // padding
        let line = [];
        let lineNumber = 0;
        const lineHeight = 1.1; // ems
        let tspan = text
          .append("tspan")
          .attr("x", 4)
          .attr("y", 14)
          .attr("dy", `${lineNumber * lineHeight}em`);

        for (let word of words) {
          line.push(word);
          tspan.text(line.join(" "));
          if (tspan.node().getComputedTextLength() > boxWidth) {
            line.pop(); // remove last word
            tspan.text(line.join(" "));
            line = [word]; // start new line
            lineNumber += 1;
            tspan = text
              .append("tspan")
              .attr("x", 4)
              .attr("y", 14)
              .attr("dy", `${lineNumber * lineHeight}em`)
              .text(word);
          }
        }
      });
  }, [data, dimensions]);

  return (
    <div
      ref={containerRef}
      style={{ width: "100%", height: "80vh", position: "relative" }}
    >
      <svg ref={svgRef} style={{ width: "100%", height: "95%" }}></svg>
      <div
        ref={tooltipRef}
        className="overflow-x-hidden "
        style={{ position: "absolute", visibility: "hidden" }}
      ></div>
    </div>
  );
};

export default Treemap;
