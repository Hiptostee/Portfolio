import { NextResponse } from "next/server";
import {
  getAzureOpenAIClient,
  AZURE_CONFIG,
} from "../../../../lib/azureOpenAI";

export const maxDuration = 60;
export const dynamic = "force-dynamic";

export async function POST(request) {
  try {
    const { fullText, question, messages } = await request.json();

    if (!fullText || !question) {
      return NextResponse.json(
        { error: "Missing fullText or question in the request body." },
        { status: 400 }
      );
    }

    const client = getAzureOpenAIClient();

    const stream = await client.chat.completions.create({
      model: AZURE_CONFIG.deployment,
      messages: [
        {
          role: "system",
          content:
            "You are an expert in answering questions about research papers, financial documents, and congressional bills. Your primary goal is to provide accurate, concise answers derived STRICTLY from the provided context (the full text of the document). Do not use outside knowledge. If the answer cannot be found in the provided text, state that clearly. Use markdown for formatting when appropriate.",
        },
        {
          role: "user",
          content: `Using the following document text as the primary source: ###${fullText}###. Now, considering the previous conversation history for context: ###${JSON.stringify(
            messages
          )}###, answer the following question: ###${question}###.`,
        },
      ],
      stream: true,
    });

    const readableStream = new ReadableStream({
      async start(controller) {
        const encoder = new TextEncoder();
        for await (const chunk of stream) {
          const content = chunk.choices[0]?.delta?.content || "";
          if (content) {
            controller.enqueue(encoder.encode(content));
          }
        }
        controller.close();
      },
      cancel() {
        console.log("Stream cancelled by client.");
      },
    });

    return new Response(readableStream, {
      headers: {
        "Content-Type": "text/plain; charset=utf-8",
        "X-Content-Type-Options": "nosniff",
      },
    });
  } catch (error) {
    console.error("Error in followUp API:", error);
    const errorMessage = error.message || "An unexpected error occurred.";
    return NextResponse.json({ error: errorMessage }, { status: 500 });
  }
}
