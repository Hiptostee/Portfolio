"use client";
import React, { useEffect, useRef } from "react";
import * as d3 from "d3";
import { useRouter } from "next/navigation";

const CircularPacking = ({ data, onClickSector }) => {
  const router = useRouter();
  const svgRef = useRef(null);
  const tooltipRef = useRef(null);

  useEffect(() => {
    const width = 800;
    const height = 800;

    // Create D3 hierarchy and pack layout
    const root = d3
      .hierarchy({ children: data })
      .sum((d) => d.marketCap)
      .sort((a, b) => b.value - a.value);

    const pack = d3.pack().size([width, height]).padding(3);
    const nodes = pack(root).leaves();

    // Create SVG element
    const svg = d3
      .select(svgRef.current)
      .attr("width", width)
      .attr("height", height)
      .style("background", "transparent");

    // Create tooltip
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

    // Determine min and max ESG scores
    const esgScores = data.map((d) => d.esgScore);
    const minEsgScore = d3.min(esgScores);
    const maxEsgScore = d3.max(esgScores);

    // Create color scale
    const colorScale = d3
      .scaleLinear()
      .domain([minEsgScore, maxEsgScore])
      .range(["#00b241", "#950202"]); // Light green to red

    // Draw circles with interactions
    svg
      .selectAll("circle")
      .data(nodes)
      .enter()
      .append("circle")
      .attr("cx", (d) => d.x)
      .attr("cy", (d) => d.y)
      .attr("r", (d) => d.r)
      .style("fill", (d) => colorScale(d.data.esgScore)) // Use color scale
      .style("stroke-width", 2)
      .on("mouseover", function (event, d) {
        tooltip
          .style("visibility", "visible")
          .text(
            `Avg ESG: ${d.data.esgScore}, Market Cap (trillions of USD): ${d.data.marketCap}`
          );
        d3.select(this)
          .transition()
          .duration(200)
          .style(
            "fill",
            d3.color(colorScale(d.data.esgScore)).darker(0.3).toString()
          );
      })
      .on("mousemove", (event) => {
        tooltip
          .style("top", event.pageY + "px")
          .style("left", event.pageX + "px");
      })
      .on("mouseout", function () {
        tooltip.style("visibility", "hidden");
        d3.select(this)
          .transition()
          .duration(200)
          .style("fill", (d) => colorScale(d.data.esgScore));
      })
      .on("click", (event, d) => {
        if (onClickSector) {
          router.push(`/sector?sector=${d.data.sector}`);
        }
      });

    // Add sector labels
    svg
      .selectAll("text")
      .data(nodes)
      .enter()
      .append("text")
      .attr("x", (d) => d.x)
      .attr("y", (d) => d.y)
      .attr("text-anchor", "middle")
      .attr("dy", ".35em")
      .text((d) => d.data.sector)
      .style("fill", "#fff")
      .style("font-size", (d) => Math.min(d.r / 2, 12) + "px")
      .style("pointer-events", "none");
  }, [data]);

  return (
    <>
      <svg ref={svgRef}></svg>
      <div
        ref={tooltipRef}
        style={{ position: "absolute", visibility: "hidden" }}
      ></div>
    </>
  );
};

export default CircularPacking;
